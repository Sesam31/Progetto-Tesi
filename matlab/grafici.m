clear;
Array = csvread('~/circle3.csv');
col1=Array(:,1);
col2=Array(:,2);
col3=Array(:,3);
col4=Array(:,4);
xlabel('Posizione X [mm]')
ylabel('Posizione Y [mm]')
hold on 
plot(col3,col4,'r.')
%plot(col1,col2,'r-','LineWidth',3.0)
grid on
%title('Setpoint in movimento circolare')