# About
A flocking model based on Craig Reynold's Boids program with behaviour for flocking and formations. Developed as part of COSC3500 course at the Univerity of Queensland. It is optimised to run in O(N) using spatial subdivision and parallelised for GPU.

# Formations / Flocking
<img src="https://github.com/user-attachments/assets/937c9a40-ab83-4cc2-8d8d-1c7084b7cda5" align="left" width="400px"/>
My extension of the original boids model to simulate the flocking and formation behaviour found in birds. Birds will pull alongside and behind other birds to conserve energy during flight.
I also wanted to stay true to the original boids algorithm where each boid uses only the state of other boids around it to inform its direction. Default boids use cohesion to go towards the mean position of nearby boids. In this new formation behaviour, instead boids will try to fall in line along an angled vector DV if the centre of mass of other boids is in front of them.
<br clear="left"/>

# Usage
flockingCPU.cpp may be run with any c++ compiler.

flockingGPU.cu is orders of magnitude faster but requires a CUDA compatible GPU.

The results are outputted to text file "data.txt".

To visualise results and save as a GIF, run the python program visualiser.py.

# Examples
<img src="https://github.com/user-attachments/assets/3bdc74ac-95cb-4077-aa4e-ad0701c47d06" width="500">
<img src="https://github.com/user-attachments/assets/c1cedd17-2dab-44c0-b43e-8ae58ce9f7f8" width="500">
