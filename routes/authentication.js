const express    = require('express');
const router     = express.Router();

const firebase = require('firebase');
require('firebase/auth');
global.userIds = []; // currently logged in users
let whichRedirect = ''; // will tell where this was redirected from --> correct toastMessage

// Login index page
router.get('/login', (req, res) => {
  // Rendering based on where the request came from
  if (whichRedirect === '/auth/login') {
    res.render('login', {
      wrongUserPass: true,
      toastMessage: 'Email/Password is incorrect.'
    });
  } else if (whichRedirect === '/auth/logout') {
    res.render('login', {
      signedOut: true,
      toastMessage: 'Logged out successfully.'
    });
  } else if (whichRedirect === '/resetpassword') {
    res.render('login', {
      signedOut: true,
      toastMessage: 'Reset password email sent successfully..'
    });
  } else {
    res.render('login');
  }
  whichRedirect = ''; // reset the value
});

// Login post function
router.post('/login', (req, res) => {
  firebase.auth().signInWithEmailAndPassword(req.body.email, req.body.password)
    .then((user) => {
      userIds.push(firebase.auth().currentUser.uid);
      res.redirect('../../dashboard/' + firebase.auth().currentUser.uid);
    })
    .catch(function(error) {
      // Handle Errors here.
      whichRedirect = '/auth/login';
      res.redirect('/auth/login');
    });
});

// Logout post function
router.post('/logout', (req, res) => {
  // close ros connection and turn off all nodes
  if(ros !== null) {
      ros.close();
      turnOffAllNodes();
  }
  // Remove user from signed-in users list (tested successfully)
  let index = userIds.indexOf(req.body.uid);
  if (index > -1) {
    userIds.splice(index, 1);
  }
  // Sign out user
  firebase.auth().signOut()
    .then(() => {
      // sign out
      console.log('signout success');
      whichRedirect = '/auth/logout';
      res.redirect('/auth/login');
    })
    .catch(function(err) {
      // Handle errors
      console.log('signout error.');
    });
});

/* Reset password GET and POST requests */
router.get('/resetpassword', (req, res) => {
  res.render('resetpassword');
});
router.post('/resetpassword', (req, res) => {
  // Firebase reset password instructions
  firebase.auth().sendPasswordResetEmail(req.body.email).then(function() {
    // Email sent. Re-route to login with a toastr notifcation
    console.log(`Email sent to ${req.body.email} successfully.`);
    whichRedirect = '/resetpassword';
    res.redirect('login');
  }).catch(function(error) {
    // An error happened.
    console.log('an error occured');
  });
});

/*-------------------- Finally, export auth router --------------------*/
module.exports = router;
