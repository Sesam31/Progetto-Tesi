clear all
C = csvread( '~/Documents/Progetto-Tesi/csv/step.csv' )
col1=C(:,2);
col2=C(:,3);
xlabel('Tempo [ms]')
ylabel('Posizione X [mm]')
t=0:100:100*(length(col2)-1);
hold on
plot(t,col2,'k','LineWidth',2)
plot(t,col1,'r','LineWidth',2)
fontsize=20;
set([gca; findall(gca, 'Type','text')], 'FontSize', fontsize);
grid on