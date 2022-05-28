# CS-E5520 Advanced Computer Graphics, Spring 2022
# Lehtinen / Kemppinen, Timonen
#
# Assignment 2: Radiosity

R1 Integrate old raytracing code (1p): done
R2 Area light source             (3p): done
R3 Radiosity                     (6p): done

+

Visualizing bounces separately: {
There are two view modes: final result (V) and bounce only (B).
In bounce only mode, set the bounce you want to display using the slider.
Then, when you press compute radiosity, the results of the specified bounce will be displayed.
}

Sobol: {
In light sampling and indirect bounces.
getRandomPointHalfSphere in Radiosity.cpp
AreaLight::sample in AreaLight.cpp
Sobol code in sobol.hpp and sobol.cpp. Source: https://gruenschloss.org/
}

Stratified sampling for the area light
