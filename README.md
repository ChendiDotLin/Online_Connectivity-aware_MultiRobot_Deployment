# Overview

This is the demonstration work of my master thesis: https://www.ri.cmu.edu/publications/online-connectivity-aware-dynamic-distribution-for-heterogeneous-multi-robot/. It is based on the framework of Behavior Mixing and dynamically distribute the heterogeneous robots both on the allocation level and the control level. 

# MATLAB Version 

Currently, we know that the simulator works with MATLAB 2018b and higher.  The backwards compatibility issues are mostly due to changes in the way MATLAB handles figures in newer releases.

# Required Toolboxes 

We make heavy use of MATLAB's optimization toolbox function 'quadprog.'  Though this toolbox isn't necessarily required, all of our barrier function algorithms utilize this function.

# robotarium-matlab-simulator
MATLAB simulator for the Robotarium!  The purpose of the Robotarium simulator is to ensure that algorithms perform reasonably well before deployment onto the Robotarium.  Note that scripts created for the simulator can be directly deployed onto the Robotarium.  To ensure minimum modification after deployment, the simulator has been created to closely approximate the actual behavior of the Robotarium's agents. 

# Usage 

First, take a look at the "examples/go_to_goal/dynamic_distribution" folder for a few, simple examples.  Note that, to run these examples, you must first run the "init.m" script to add the requisite directories.  

# Documentation 

For example mathematical documentation, FAQs, and more, visit http://www.robotarium.org.
