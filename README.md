Bode Lead Compensation Design
This MATLAB function bode_lead.m implements a phase-lead compensator design method using the Bode plot. It calculates the lead compensation parameters K, T, and alpha to achieve a specified phase margin (PM) at a crossover frequency (wc) for a given plant transfer function np/dp.

Usage

[K,T,alpha] = bode_lead(np, dp, wc, PM);
np: Numerator polynomial of the plant transfer function.
dp: Denominator polynomial of the plant transfer function.
wc: Crossover frequency (in radians/second).
PM: Desired phase margin (in degrees).
The function computes the lead compensation parameters K, T, and alpha, and plots the Bode plot of the original and compensated systems for verification.

Example

[K,T,alpha] = bode_lead([1],[1 1 0],10,60);
This example computes the lead compensation for a plant with transfer function 1/(s^2 + s), aiming for a phase margin of 60 degrees at a crossover frequency of 10 rad/s.

Note
Ensure that the plant transfer function np/dp is in the proper form and that the desired phase margin can be achieved with a phase-lead compensator.

Feel free to add more sections or details as needed for your specific application or audience.
