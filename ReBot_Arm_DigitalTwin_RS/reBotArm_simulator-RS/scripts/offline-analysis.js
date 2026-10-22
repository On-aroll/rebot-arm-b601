#!/usr/bin/env node
'use strict';
// Pure file processing. No ROS, SDK, server, camera or network imports.
const fs=require('node:fs');
const vision=require('../public/js/vision-replay.js');
const gripper=require('../public/js/grasp-recording.js');
try{
  const [kind,input]=process.argv.slice(2);
  if(!['vision','j7','example'].includes(kind)||kind!=='example'&&!input)throw new Error('用法: node scripts/offline-analysis.js example | vision 文件.json | j7 文件.json');
  if(input&&fs.statSync(input).size>8*1024*1024)throw new Error('输入须小于8MB');
  const value=kind==='example'?vision.example():JSON.parse(fs.readFileSync(input,'utf8'));
  const result=kind==='vision'?vision.review(value):kind==='j7'?gripper.analyze(value):value;
  process.stdout.write(JSON.stringify(result,null,2)+'\n');
}catch(e){process.stderr.write(e.message+'\n');process.exitCode=1;}
