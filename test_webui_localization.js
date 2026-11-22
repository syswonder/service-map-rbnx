// Backend snapshots are the only source of localization/candidate state.
const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const source = fs.readFileSync(__dirname + '/src/mapping_rbnx/webui_static/app.js', 'utf8');
const progress = source.slice(source.indexOf('function localizationProgress('), source.indexOf('setInterval(renderLocalizationProgress'));
const candidates = source.slice(source.indexOf('function syncCandidates('), source.indexOf('function togglePanel('));
const trust = source.slice(source.indexOf('function applyTrust('), source.indexOf('function setPose('));
const elements = {};
let opens = 0, requests = 0;
const element = () => ({dataset: {}, children: [], append(...items) {this.children.push(...items);},
  appendChild(item) {this.children.push(item);}, replaceChildren() {this.children = [];}, addEventListener() {}});
const ctx = vm.createContext({STALE: false, CURLOC: {}, CURMODE: 'localization', CURENGINE: 'slam_toolbox', CANDIDATES: [],
  CANDIDATE_MAP: null, CANDIDATE_SIGNATURE: '', OPENED_CANDIDATE_RUN: null, SUBMITTING_POSE: false,
  document: {getElementById: id => elements[id] ||= element(), createElement: element},
  paneOpen: () => false, togglePanel: () => opens++, askConfirm: async () => true,
  fetch: async (_, options) => { requests++; assert.equal(JSON.parse(options.body).run_id, 3); return {json: async () => ({ok:true})}; },
  poll: async () => {}, setStatus: () => {},
});
vm.runInContext(progress + candidates + trust, ctx);
elements.trust = {classList: {add() {}, remove() {}}};
ctx.CURTRUST = {verdict:'ok', fit:1, threshold:.6, detail:'27/27 scored returns hit walls'};
ctx.applyTrust();
assert.match(elements.trusttext.innerHTML, /100% of scored returns/);
assert.match(elements.trusttext.innerHTML, /Several poses can match/);
assert.equal(elements.trust.title, ctx.CURTRUST.detail);
const pose = {rank:1, x:1, y:2, theta:0, score:.8};
const state = (phase, run_id=3) => ({engine:'slam_toolbox',map_id:'lab',generation:2,localization:{map_id:'lab',generation:2,
  phase,run_id,candidates:[pose],detail:'current run',filter_state:'running',phase_started_at:(Date.now()-25000)/1000}});
ctx.syncCandidates(state('choosing'));
assert.equal(ctx.CANDIDATES.length, 1);
assert.match(elements['candidate-list'].children[0].children[0].textContent, /scan match 80\/100/);
assert.equal(opens, 1);
ctx.syncCandidates(state('choosing'));
assert.equal(opens, 1, 'polling must not reopen a panel the user closed');
assert.equal(requests, 0, 'rendering candidates must never issue a search request');
ctx.CURLOC = state('searching').localization;
ctx.renderLocalizationProgress();
assert.match(elements['loc-progress-time'].textContent, /25 s/);
assert.equal(elements['loc-progress-spinner'].hidden, false);
ctx.CURLOC = state('choosing').localization;
ctx.renderLocalizationProgress();
assert.match(elements['candidate-filter-status'].textContent, /active/);
ctx.syncCandidates(state('ready'));
assert.equal(ctx.CANDIDATES.length, 0, 'completion must remove candidates even if a stale list was included');
assert.equal(elements['candidate-list'].children.length, 0);
ctx.syncCandidates({...state('choosing'), generation:9});
assert.equal(ctx.CANDIDATES.length, 0, 'old map generation cannot offer candidates');
ctx.syncCandidates({...state('choosing'), engine:'rtabmap'});
assert.equal(ctx.CANDIDATES.length, 0, 'RTAB-Map must never offer initial localization candidates');
ctx.syncCandidates(state('choosing'));
(async () => {
  await ctx.chooseCandidate(pose);
  assert.equal(requests, 1, 'a choice submits only its pose and run identity');
  assert.equal(ctx.SUBMITTING_POSE, false);
  ctx.CURLOC = state('ready').localization;
  assert.equal(ctx.localizationProgress().key, 'ready');
  ctx.STALE = true;
  assert.equal(ctx.localizationProgress().key, 'offline');
  console.log('PASS: shared run snapshots, cached candidates, map scope, completion clearing, phase timer and selection identity');
})().catch(error => {console.error(error); process.exitCode=1;});
