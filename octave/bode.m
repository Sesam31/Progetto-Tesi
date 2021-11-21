clear all
pkg load control;
s=tf('s');
g=1/s^2;
bode(g);
fontsize=25;
set([gca; findall(gca, 'Type','text')], 'FontSize', fontsize);
