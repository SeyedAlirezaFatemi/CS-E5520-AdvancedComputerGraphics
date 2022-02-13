# CS-E5520 Advanced Computer Graphics, Spring 2022
# Lehtinen / Kemppinen, Timonen
#
# Assignment 1: Accelerated Ray Tracing

Student name: Seyed Alireza FatemiJahromi

R1 BVH construction and traversal (5p): done
        R2 BVH saving and loading (1p): done - Turn on by setting the variable at line 904 in App.cpp. It works correctly.
              R3 Simple texturing (1p): done
             R4 Ambient occlusion (2p): done
         R5 Simple multithreading (1p): done

# Did you do any extra credit work?
Alpha and specular textures (both)
Jitter supersampling
Tangent space normal mapping
BVH Surface Area Heuristic
Efficient SAH building
Optimize your tracer (Early exits in tree traversal and intersection routines + Choose the minimum triangle count of the leaf nodes on your BVH carefully)


SAH is used by default. I didn't have time to add a button for it. The implementation starts at line 142 in RayTracer.cpp.
Changing if (splitMode == SplitMode_Sah) to if (false) will switch to Spatial Median.
The construction is O(n). Using this instead of Spatial Median doubles the speed in the crytek-sponza scene.
