# AI-and-Multi-Agent
KTH course project: https://www.kth.se/student/kurser/kurs/DD2438?l=en

**Project 1:** plan and execute a path from start to goal for both car and drone in Unity. The objective of Assignment 1 is that you get familiar with the car and drone motion models, and understand how to plan and execute a path in a given environment using a fairly complex car model.

**Project 2:** In Assignment 2, all problems are set in a Maze of the same type as Assignment 1. However, now there are a large set of cars/drones in the same environment, with different goal locations.

**Project3:** In Assignment 3, all problems are set in a Maze of the same type as Assignment 2, but this time you will be competing against the other teams in a version of the **Pac Man Capture the Flag** game.

> The rules are as follows:
>
> - The maze will have two symmetric halves, the blue and red sides.
> - Each team spawns on their own side, as ghosts (that can kill pac man)
> - Once you travel into the side of the other team, your ghost transforms into a pac man.
> - On the other side, you can eat pills (as pac man)
> - If you come back to your side after having eaten some enemy pills, these pills will be dropped on your side close to where you crossed the border
> - The objective is to have more pills on your side
> - If you eat a super-pill on the opponent side, you can eat ghosts for a short amount of time (10s)
> - If you try to eat a super-pill on your side you die
> - If two opposing agents collide while standing on opposite sides of the maze (both being pac men or both being ghosts), both die.
> - All killed agents will
>   - immediately drop all pills they are carrying
>   - re-spawn on their own side
> - Sensing of enemy agent works as follows
>   - if an enemy is within line-of-sight of one agent on your team, you get access to its position
>   - if an enemy is not within line-of-sight, but moving above 0.3m/s you can "hear" him, in the sense of getting a distorted location (+/-5) the distortion will be re-sampled every (1) second
>   - if one of your pills get eaten, an enemy was there...
