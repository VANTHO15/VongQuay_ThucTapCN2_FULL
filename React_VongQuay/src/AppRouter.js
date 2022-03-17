import React from 'react';
import './App.css'
import {
  BrowserRouter as Router,
  Switch,
  Route,
  Redirect
} from "react-router-dom";
import Login from './Components/Login';
import Main from './Components/Main';
import Main1 from './Components/Main1';
import Account from './Components/Account';

export function AppRouter() {
  return (
    <Router>
        <Switch>
            <Route path="/Main" render={()=>{
                if(localStorage.getItem("AccessToken"))
                {
                    return <Main></Main>
                }
                else
                {
                    <Redirect to="/"></Redirect>
                }
            }}>
            </Route>
            <Route path="/Main1" render={()=>{
                if(localStorage.getItem("AccessToken"))
                {
                    return <Main1></Main1>
                }
                else
                {
                    <Redirect to="/"></Redirect>
                }
            }}>
            </Route>
            <Route path="/Change">
               <Account></Account>
            </Route>

            <Route path="/">
                <Login></Login>
            </Route>

        </Switch>
    </Router>
  );
};