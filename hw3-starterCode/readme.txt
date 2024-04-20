Assignment #3: Ray tracing

FULL NAME: Shidong  Zhang


MANDATORY FEATURES
------------------

<Under "Status" please indicate whether it has been implemented and is
functioning correctly.  If not, please explain the current status.>

Feature:                                 Status: finish? (yes/no)
-------------------------------------    -------------------------
1) Ray tracing triangles                  yes

2) Ray tracing sphere                     yes

3) Triangle Phong Shading            yes

4) Sphere Phong Shading              yes

5) Shadows rays                             yes

6) Still images                                yes
   
7) Extra Credit (up to 30 points)

	1. Recursive reflection: Implement recursive reflection, just modify the attenuation and numRecursive in code, for testing attenuation=0.8, numRecursive=2.
	
	2. Antialiasing: Implement antialiasing by average the value of subpixels, just modify numAntialiasing in code, for testing, numAntialiasing=3, so that we subplot pixel to 3x3=9 subpixels
	
	3. Soft shadow: Implement soft shadow by generate sublights in a sphere area of original light, juse modify numSoftShadow and radiusSoftShadow in code, for testing, numSoftShadow=30, radiusSoftShadow=0.5.
	
	4. MonteCarlo sampling: Implement the monte carlo sampling, just modify numMonteCarlo and set useMonteCarlo to true in code, for testing, numMonteCarlo=200.
	
	5. Threads in MonteCarlo sampling: Using multithreading for improving the performace, just change numThreads in code, defalt the value is 8.

	6. BVH tree implementation: For further speed up the calculation, I implement BVH (Bounding volume hierarchy) and it does speed up alot, for siggraph.scene, usually the time is 13.8 seconds, and the total triangle intersection functions run 1219784770 times, while by using BVH, the total time is 0.6 seconds and there are just 7331583 triangle intersection and 36625324 AABB box intersections, which is a 20 times faster.

	7. I also create a new scene spherePyramidM.scene in the scene folder, the result is shown in otherImages\spherePyramidMonteCarlo.jpg.

The still images are shown in images folder, and other more detailed results are in otherImages folder, the scene folder store the scenes, scene end with 'M' means this scene is for monte carlo sampling.

siggraph.jpg - The result of siggraph.scene
siggraphMonteCarlo.jpg - The result of siggraphM.scene
siggraphAntiSoftRecur.jpg - The result of siggraph.scene using antialiasing, soft shadow and recursive reflection.
snow.jpg - The result of snowman.scene
test1.jpg - The result of test1.scene
spheres.jpg - The result of spheres.scene
table.jpg - The result of table.scene
tableAntiSoftRecur.jpg -  table.scene using antialiasing, soft shadow and recursive reflection.
test2.jpg - The result test2.scene
toy.jpg - The result of toys.scene


Images in otherImages folder:

snowMonteCarlo.jpg - snowM.scene using monte carlo sampling.
spherePyramidMonteCarlo.jpg - spherePyramidM.scene using monte carlo samlping.
spheresAntiSoftRecur.jpg - spheres.scene using antialiasing, soft shadow and recursive reflection.
spheresMonteCarlo.jpg - spheresM.scene using monte carlo sampling
toyAntiSoftRecur.jpg - toy.scene using antialiasing, soft shadow and recursive reflection.
tableAnti.jpg - table.scene using antialiasing.
tableSoft.jpg - table.scene using softshadow.
tableRecur.jpg - table.scene using recursive reflection.
tableMonteCarlo.jpg - tableM.scene using monte carlo sampling
test2AntiSoftRecur.jpg - test2.scene using antialiasing, softshadow and recursive reflection.
test2M2560.jpg - 1920x1360 higher resolution for test2M.scene
test2M10000.jpg - Sampled 10000 times for test2M.scene
