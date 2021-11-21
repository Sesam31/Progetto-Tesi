clear

x=0:0.001:2*pi;
plot(cos(x),cos(2*x),'r','LineWidth', 3);
xlabel("cos(x)");ylabel("cos(2x)");
fontsize=25;
set([gca; findall(gca, 'Type','text')], 'FontSize', fontsize);
saveas(1, "lissajous4.png");
