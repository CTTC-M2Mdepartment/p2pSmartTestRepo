# p2pSmartTestRepo
Code for the NS3 simulation of a complex p2pSmartTest scenario using LTE and WiFi

## how to use it
In this repo there are only the required files to perform a simulation with the already-modified NS3 version at JUNO. If you try otherwise, you'd must likely get an error regarding traced values and callbacks.

Having cleared the above, you use the executioner python script to define scenarios. These variables will then be passed as arguments to waf, and therefore to the NS3 simulation file.

L
