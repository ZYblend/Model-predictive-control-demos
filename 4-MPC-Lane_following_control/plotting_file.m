%% Plotting file

x_actual = out.logsout.getElement('x_actual').Values.Data;
y_actual = out.logsout.getElement('y_actual').Values.Data;

e1 = out.logsout.getElement('crosstrack error').Values.Data;
e2 = out.logsout.getElement('heading error').Values.Data;
t = out.logsout.getElement('heading error').Values.Time;
steer = out.logsout.getElement('steering').Values.Data;

Vx = out.logsout.getElement('longitudinal velocity').Values.Data;


%% plot trajectory
if double_lane_flag == 1
    fplot(g1, [x0d, x_boundary] ,'ro')
    hold on, fplot(g2, [x_boundary, 100],'ro')
else
    fplot(g1, [0,150],'ro')
end

%% actual
hold on, plot(x_actual,y_actual,'k','LineWidth',1)
xlabel('x','FontWeight','bold')
ylabel('y','FontWeight','bold')
legend('reference','actual')
title('tracking performance')

figure (2)
subplot(2,1,1)
plot(t,e1,'k','LineWidth',1);
title('tracking error')
legend('crosstrack error')
subplot(2,1,2)
plot(t,e2,'k','LineWidth',1);
legend('heading error')

figure (3)
yline(desired_longitudinal_speed,'r','LineWidth',1);
hold on,plot(Vx,'k','LineWidth',1);
title('cruise control result')

figure (4)
plot(t,steer,'k','LineWidth',1);
title('steering angle')

