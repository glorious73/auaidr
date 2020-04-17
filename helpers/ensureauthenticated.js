/* The following [exported function] is meant to protect the routes by ensuring that the user has authenticated */
const firebase   = require('firebase');
require('firebase/auth');

module.exports = {
  ensureAuthenticated: function(req, res, next) {
    // get hold of the user id (if it exists)
    let userIndex = userIds.indexOf(req.url.substring(req.url.lastIndexOf('/')+1));
    console.log(`UserIndex = ${userIndex}. Id = ${req.url.substring(req.url.lastIndexOf('/')+1)}`);
    // get hold of the current user (if exists)
    let user = firebase.auth().currentUser;
    // check if whether we have a current user and/or there is an id
      if (user && !(userIndex > -1)) {
        req.user = user;
        next();
      } else {
        // check if we have another signed-in user and refresh the page for him/her
        if(userIndex >-1) {
          next();
        } else {
          res.redirect('/auth/login');
        }
      }
  }
}
