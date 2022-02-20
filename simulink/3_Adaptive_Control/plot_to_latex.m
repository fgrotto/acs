figure();
subplot(1,2,1);
hold on;
grid on;
plot(out.q.Time, out.q.data(:,1),'LineWidth', 1.2);
hold on, plot(out.q.Time, out.qd.data(:,1),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('q [rad/s]');
legend('q', 'qd');

subplot(1,2,2);
hold on;
grid on;
plot(out.q.Time, out.param.data(:,1),'LineWidth', 1.2);
hold on, plot(out.q.Time,out.param.data(:,2),'LineWidth', 1.2);
hold on; plot(out.q.Time,out.param.data(:,3),'LineWidth', 1.2);
yline(0.1,'-','I');
yline(1.9620,'-','G');
yline(0.2,'-','F');
xlabel('time [s]');
ylabel('parameters');
legend('I\_hat', 'F\_hat', 'G\_hat');