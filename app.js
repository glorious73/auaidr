/* ---------- Main JavaScript file ----------*/
// Modules
const express    = require('express');
const exphbs     = require('express-handlebars');
const path       = require('path');
const bodyParser = require('body-parser');
const cors       = require('cors');
// auth
const authRoute      = require('./routes/authentication');
const firebase       = require('firebase');
const {ensureAuthenticated} = require('./helpers/ensureauthenticated'); // used to protect routes
require('firebase/auth');
// ROS
const rosRoute = require('./routes/rosbackend');

// Variables and constants
const port = process.env.PORT || 3000;
/* ---------- Initialization ---------- */
// initialize the express application
const app = express();

/*----- Enable CORS ----*/
app.use(function(req, res, next) {
  res.header("Access-Control-Allow-Origin", "*");
  res.header('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE');
  res.header('Access-Control-Allow-Headers', 'Content-Type');
  next();
});
/*----- Public folder -----*/
app.use(express.static(path.join(__dirname, 'public')));
/*----- /Public folder -----*/
/*-------------------- Middleware --------------------*/
/*----- firebase middleware -----*/
const config = {
  apiKey: "AIzaSyBpzwC7ztvKP3r1b5vHpHJoaHL0HNy5f08",
  authDomain: "aidr-alfaisal.firebaseapp.com",
  databaseURL: "https://aidr-alfaisal.firebaseio.com",
  projectId: "aidr-alfaisal",
  storageBucket: "aidr-alfaisal.appspot.com",
  messagingSenderId: "165306467358"
};
firebase.initializeApp(config);
/*----- handlebars middleware -----*/
app.engine('handlebars', exphbs({
  defaultLayout: 'main'
}));
app.set('view engine', 'handlebars');
/*----- Body parser middleware -----*/
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({extended: false}));

/*----- Cors for cross-domain functionality -----*/
app.use(cors());
/*-------------------- End of middleware --------------------*/

/*-------------------- Requests --------------------*/
// index page
app.get('/', (req, res) => {
    res.redirect('/auth/login');
});
// index page after sign in successful
app.get('/dashboard/:id', ensureAuthenticated, (req, res) => {
  res.render('index', {
    userId: req.url.substring(req.url.lastIndexOf('/')+1)
  });
});
// normal dashboard (without id) to redirect to login page
app.get('/dashboard', (req, res) => {
  res.redirect('/auth/login');
})
/*-------------------- End of Requests --------------------*/

/*----- Specific routes -----*/
app.use('/auth', authRoute);
app.use('/ros', rosRoute);

// Setup listen for server
app.listen(port, () => console.log(`AIDR is listening on port ${port}.`));
