clear all
clear figure
C = csvread( '~/Documents/Progetto-Tesi/csv/u1.csv' )
col1=C(:,1);
col2=C(:,2);
col3=C(:,3);
col4=C(:,4);
xlabel('Posizione X [mm]')
ylabel('Posizione Y [mm]')
hold on 
scatter(col3,col4,'k','o','LineWidth',2)
plot(col1,col2,'r*','LineWidth',2)
axis("equal");
fontsize=20;
set([gca; findall(gca, 'Type','text')], 'FontSize', fontsize);
grid on
