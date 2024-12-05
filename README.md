# CS4080 Group Project: Kinematic Collision Solver

## Overview

The Kinematic Collision Solver is a C++ library designed to handle collisions between arbitrary convex shapes. The library utilizes the Separating Axis Theorem (SAT) to check for intersections and computes the time of collision between shapes based on their movement. It supports a variety of 2D shapes, and it's optimized for performance to handle complex scenarios involving multiple shapes.

The project offers a solid framework for simulating and resolving collisions in a 2D space; making it perfect for physics engines, game development, and simulations involving rigid body dynamics.

## Collaborators

- Luke Lewis
- Erick Hambardzumyan

## Project Description

This library provides the necessary tools to detect and handle collisions between convex shapes in a 2D environment. It features efficient collision detection algorithms and a kinematic solver for moving objects. The primary function of the project is based on the Separating Axis Theorem (SAT), which is used to test whether two convex shapes overlap, and to compute the collision response.

The solver also handles the concept of move and slide, where an object that collides with another will slide along the collision surface to avoid further overlap, maintaining its velocity while resolving collisions.

## Features

Collision Detection: Tests if two convex shapes are colliding using the Separating Axis Theorem (SAT).

Kinematic Collision Solver: Computes how objects move and slide when a collision occurs, updating their positions and velocities.

Support for Convex Shapes: Handles basic convex shapes such as polygons.
Efficient Performance: Optimized for detecting collisions and handling large numbers of objects.

Collision Normal Calculation: Computes the normal vector of the collision surface to resolve the response.

Projection of Shapes: Projects shapes onto an axis to test for overlap during collision detection.
