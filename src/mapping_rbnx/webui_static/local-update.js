"use strict";
(async () => {
  const source = new URLSearchParams(location.search).get("map_id");
  const get = id => document.getElementById(id);
  const status = message => { get("status").textContent = message; };
  const canvas = get("map"), context = canvas.getContext("2d");
  let meta, image, anchor, selection, derived, busy=false, started=false;
  const point = event => {
    const r = canvas.getBoundingClientRect();
    return [Math.max(0, Math.min(canvas.width, (event.clientX-r.left)*canvas.width/r.width)),
      Math.max(0, Math.min(canvas.height, (event.clientY-r.top)*canvas.height/r.height))];
  };
  const bounds = () => [meta.origin[0]+selection[0]*meta.resolution,
    meta.origin[1]+(meta.height-selection[3])*meta.resolution,
    meta.origin[0]+selection[2]*meta.resolution,
    meta.origin[1]+(meta.height-selection[1])*meta.resolution];
  const redraw = () => {
    context.drawImage(image, 0, 0);
    if (selection) {
      context.fillStyle="rgba(227,94,52,.18)"; context.strokeStyle="#df502d";context.lineWidth=2;
      context.fillRect(selection[0], selection[1], selection[2]-selection[0], selection[3]-selection[1]);
      context.strokeRect(selection[0], selection[1], selection[2]-selection[0], selection[3]-selection[1]);
      get("bounds").textContent = "选区（米）："+bounds().map(v=>v.toFixed(2)).join(", ");
    }
    get("prepare").disabled = busy || !selection || selection[2]-selection[0]<2 || selection[3]-selection[1]<2 || !!derived;
  };
  async function post(url, body) {
    const response = await fetch(url, {method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(body)});
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.detail || "请求失败");
    return result;
  }
  canvas.onpointerdown = event => {
    if (derived || busy || !meta || !image) return;
    anchor=point(event);canvas.setPointerCapture(event.pointerId);
  };
  canvas.onpointermove = event => {
    if (!anchor) return;
    const p=point(event);selection=[Math.min(anchor[0],p[0]),Math.min(anchor[1],p[1]),Math.max(anchor[0],p[0]),Math.max(anchor[1],p[1])];redraw();
  };
  canvas.onpointerup = canvas.onpointercancel = () => { anchor=null; };
  get("prepare").onclick = async () => {
    busy=true;anchor=null;get("prepare").disabled=true;status("正在创建并校验地图分支…");
    try {
      const result = await post("/api/mapping/local_update", {base_map_id:source,map_id:get("new-id").value,bounds:bounds()});
      derived=result.map_id;get("new-id").value=derived;get("new-id").disabled=true;
      status("已创建："+derived+"。请确认机器人位于保留区域，再开始定位和增量建图。");
      get("start").disabled=!get("ready").checked;
    } catch(error) { status(error.message); }
    finally { busy=false;redraw(); }
  };
  get("ready").onchange=()=>{get("start").disabled=started || !derived || !get("ready").checked;};
  get("start").onclick=async()=>{
    started=true;get("start").disabled=true;
    try { const result=await post("/api/mapping/start",{base_map_id:derived});status(result.detail+"\n请在主界面查看定位状态，定位通过后进入选区重建。"); }
    catch(error){status(error.message);started=false;get("start").disabled=!get("ready").checked;}
  };
  try {
    if (!source) throw new Error("缺少地图名称");
    const response=await fetch("/api/editor/meta?"+new URLSearchParams({map_id:source}));meta=await response.json();
    if (!response.ok || !meta.width) throw new Error(meta.detail || "地图读取失败");
    if (meta.version !== "original") throw new Error("该地图含独立栅格编辑，请选择未编辑的原生地图版本进行局部重建。");
    if (Math.abs(meta.origin[2])>1e-8) throw new Error("当前局部重建仅支持坐标轴对齐地图");
    get("source").textContent="原图："+source;get("new-id").value=source+"-local-"+Date.now();
    image=new Image();image.src="/api/maps/"+encodeURIComponent(source)+"/preview.png";
    await image.decode();canvas.width=meta.width;canvas.height=meta.height;redraw();
  } catch(error){status(error.message);}
})();
