(self["webpackChunk_jupyterlab_application_top"]=self["webpackChunk_jupyterlab_application_top"]||[]).push([[7796],{37796:(e,r,t)=>{"use strict";t.r(r);t.d(r,{main:()=>z});var o=t(92067);var n=t(95171);var l=t(48435);var s=t(89131);var a=t(85745);var i=t(99093);var c=t(28180);var u=t(2542);var f=t(7920);var p=t(38710);var h=t(21411);var d=t(81554);var y=t(67014);var v=t(77552);var b=t(71821);var x=t(13313);var j=t(23454);var m=t(34802);var g=t(42584);var w=t(54244);var C=t(93814);var P=t(64897);var E=t(72738);var S=t(92121);var _=t(83344);var k=t(11706);var O=t(70987);var L=t(55337);var J=t(32977);var A=t(95528);var N=t(3268);var T=t(99204);var M=t(28566);var $=t(50168);var B=t(52714);var D=t(43162);var R=t(57385);var F=t(65540);var I=t(1733);var U=t(49857);var Y=t(30124);if(Promise.allSettled===undefined){Promise.allSettled=e=>Promise.all(e.map((e=>e.then((e=>({status:"fulfilled",value:e})),(e=>({status:"rejected",reason:e}))))))}async function q(e,r){try{const t=await window._JUPYTERLAB[e].get(r);return t()}catch(t){console.warn(`Failed to create module: package: ${e}; module: ${r}`);throw t}}async function z(){var e=o.PageConfig.getOption("browserTest");if(e.toLowerCase()==="true"){var r=document.createElement("div");r.id="browserTest";document.body.appendChild(r);r.textContent="[]";r.style.display="none";var n=[];var l=false;var s=25e3;var a=function(){if(l){return}l=true;r.className="completed"};window.onerror=function(e,t,o,l,s){n.push(String(s));r.textContent=JSON.stringify(n)};console.error=function(e){n.push(String(e));r.textContent=JSON.stringify(n)}}var i=t(8775).JupyterLab;var c=[];var u=[];var f=[];var p=[];const h=[];const d=[];const y=[];const v=JSON.parse(o.PageConfig.getOption("federated_extensions"));const b=[];v.forEach((e=>{if(e.extension){b.push(e.name);h.push(q(e.name,e.extension))}if(e.mimeExtension){b.push(e.name);d.push(q(e.name,e.mimeExtension))}if(e.style){y.push(q(e.name,e.style))}}));function*x(e){let r;if(e.hasOwnProperty("__esModule")){r=e.default}else{r=e}let t=Array.isArray(r)?r:[r];for(let n of t){if(o.PageConfig.Extension.isDisabled(n.id)){c.push(n.id);continue}if(o.PageConfig.Extension.isDeferred(n.id)){u.push(n.id);f.push(n.id)}yield n}}const j=[];if(!b.includes("@jupyterlab/javascript-extension")){try{let e=t(80413);for(let r of x(e)){j.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/json-extension")){try{let e=t(14076);for(let r of x(e)){j.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/pdf-extension")){try{let e=t(75127);for(let r of x(e)){j.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/vega5-extension")){try{let e=t(82282);for(let r of x(e)){j.push(r)}}catch(E){console.error(E)}}const m=await Promise.allSettled(d);m.forEach((e=>{if(e.status==="fulfilled"){for(let r of x(e.value)){j.push(r)}}else{console.error(e.reason)}}));if(!b.includes("@jupyterlab/application-extension")){try{let e=t(12864);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/apputils-extension")){try{let e=t(85430);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/celltags-extension")){try{let e=t(27261);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/codemirror-extension")){try{let e=t(53974);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/completer-extension")){try{let e=t(45827);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/console-extension")){try{let e=t(63439);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/csvviewer-extension")){try{let e=t(37295);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/debugger-extension")){try{let e=t(22949);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/docmanager-extension")){try{let e=t(39725);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/docprovider-extension")){try{let e=t(49269);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/documentsearch-extension")){try{let e=t(56412);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/extensionmanager-extension")){try{let e=t(17645);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/filebrowser-extension")){try{let e=t(55357);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/fileeditor-extension")){try{let e=t(64082);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/help-extension")){try{let e=t(61225);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/htmlviewer-extension")){try{let e=t(38757);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/hub-extension")){try{let e=t(69594);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/imageviewer-extension")){try{let e=t(36267);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/inspector-extension")){try{let e=t(33534);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/launcher-extension")){try{let e=t(77644);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/logconsole-extension")){try{let e=t(46224);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/mainmenu-extension")){try{let e=t(15223);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/markdownviewer-extension")){try{let e=t(72400);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/mathjax2-extension")){try{let e=t(86203);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/notebook-extension")){try{let e=t(19330);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/rendermime-extension")){try{let e=t(88613);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/running-extension")){try{let e=t(80138);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/settingeditor-extension")){try{let e=t(18764);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/shortcuts-extension")){try{let e=t(62093);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/statusbar-extension")){try{let e=t(38305);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/terminal-extension")){try{let e=t(53285);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/theme-dark-extension")){try{let e=t(69588);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/theme-light-extension")){try{let e=t(53316);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/toc-extension")){try{let e=t(85080);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/tooltip-extension")){try{let e=t(22489);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/translation-extension")){try{let e=t(96644);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/ui-components-extension")){try{let e=t(57693);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}if(!b.includes("@jupyterlab/vdom-extension")){try{let e=t(48200);for(let r of x(e)){p.push(r)}}catch(E){console.error(E)}}const g=await Promise.allSettled(h);g.forEach((e=>{if(e.status==="fulfilled"){for(let r of x(e.value)){p.push(r)}}else{console.error(e.reason)}}));(await Promise.allSettled(y)).filter((({status:e})=>e==="rejected")).forEach((({reason:e})=>{console.error(e)}));const w=new i({mimeExtensions:j,disabled:{matches:c,patterns:o.PageConfig.Extension.disabled.map((function(e){return e.raw}))},deferred:{matches:u,patterns:o.PageConfig.Extension.deferred.map((function(e){return e.raw}))}});p.forEach((function(e){w.registerPluginModule(e)}));w.start({ignorePlugins:f});var C=(o.PageConfig.getOption("exposeAppInBrowser")||"").toLowerCase()==="true";var P=(o.PageConfig.getOption("devMode")||"").toLowerCase()==="true";if(C||P){window.jupyterlab=w;window.jupyterapp=w}if(e.toLowerCase()==="true"){w.restored.then((function(){a(n)})).catch((function(e){a([`RestoreError: ${e.message}`])}));window.setTimeout((function(){a(n)}),s)}}}}]);
//# sourceMappingURL=7796.135529594458df16862c.js.map?v=135529594458df16862c