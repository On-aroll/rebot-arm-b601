// Local browser boundary for every API, including config tokens and LLM proxy.
// This is not user authentication or a ROS/DDS access-control mechanism.
function allowed(req, secure=false) {
  const address=req.socket?.remoteAddress,host=req.headers.host||'';
  if(!['127.0.0.1','::1','::ffff:127.0.0.1'].includes(address) ||
     !/^(127\.0\.0\.1|localhost|\[::1\])(:\d+)?$/.test(host)) return false;
  const ownOrigin=`${secure?'https':'http'}://${host}`,origin=req.headers.origin;
  if(origin && origin!==ownOrigin) return false;
  if(['cross-site','same-site'].includes(req.headers['sec-fetch-site'])) return false;
  return ['GET','HEAD'].includes(req.method) || origin===ownOrigin;
}
module.exports={allowed};
