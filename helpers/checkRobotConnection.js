module.exports = {
  checkRobotConnection: function(req, res, next) {
    if(ros !== null) {
      next();
    } else {
      return res.json({message: 'Robot not connected.'});
    }
  }
}
