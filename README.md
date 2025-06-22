# Bachelor thesis
We worked on a system that simulated a collaborative task:

![Screenshot From 2025-06-22 14-15-39](https://github.com/user-attachments/assets/c55a0c33-88a5-4ae7-99df-a268e9c7404f)
![Screenshot From 2025-06-22 14-16-24](https://github.com/user-attachments/assets/f6a0d140-8fd7-445a-bd08-3349613f5079)

Where our job was to constrain the motion of the system to avoid bad manipulability regions, e.g. when one of the arms would stretch completely:

![Screenshot From 2025-06-22 14-17-16](https://github.com/user-attachments/assets/8cddca37-bf7b-499e-bf25-981227dfd162)

But the workspace was constrained by 3 different robots, the two physical ones, and the simulated one.
So we needed to find the coupled workspace:

![Screenshot From 2025-06-22 14-19-45](https://github.com/user-attachments/assets/3b60c2b2-8f92-4f28-b966-157fbcf8abaf)

We used sampling of different poses of the simulated robot and corresponding poses of the real robots to create a discrete representation of the coupled workspace, and the corresponding manipulability of a given pose:

![Screenshot From 2025-06-22 14-21-45](https://github.com/user-attachments/assets/7b1405cb-ca99-4cb3-8476-8d9636033864)
![Screenshot From 2025-06-22 14-22-16](https://github.com/user-attachments/assets/6b2a9dd9-619c-464a-806c-20ae6bc14d48)

Based on that we created a "virtual fixture" that would add resistance to the user when they tried moving into a bad region of the workspace:

![Screenshot From 2025-06-22 14-25-02](https://github.com/user-attachments/assets/a3f75ff3-985e-4566-b1e9-2ad77d78884a)

[Demonstation of the virtual fixture](https://youtu.be/EwlrC2Dy8Ws)

Then we did optimization of the workspace by varying the positions of the real robots relative to the one in simulation:

![Screenshot From 2025-06-22 14-26-18](https://github.com/user-attachments/assets/6b72634e-755c-4557-9f32-d10091ffb08d)

We used the avg. distance from a desired trajectory to the boundary of the workspace as a measure of how "good" a workspace was, and therefor the corresponding positions of the physical robots:

![Screenshot From 2025-06-22 14-27-36](https://github.com/user-attachments/assets/123d3026-7e10-47a9-83da-09847de2621c)

Then we tested a couple of different optimizers, namely Adam (gd), Nelder-mead (nm) and Particle swarm (pso):

![Screenshot From 2025-06-22 14-27-18](https://github.com/user-attachments/assets/da9439d9-90a6-4187-ab5a-1dd1bb9bec61)

