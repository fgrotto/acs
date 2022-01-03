figure();
subplot(2,3,1);
hold on;
grid on;
plot(out.q.Time, out.q.data(:,1),'LineWidth', 1.2);
%hold on; plot(out.qd.Time, out.qd.data(:,1),'LineWidth', 1.2);
yline(out.qd.data(1,1),'-','');
xlabel('time [s]');
ylabel('q [rad/s]');
legend('q1', 'qd1');

subplot(2,3,2);
hold on;
grid on;
plot(out.q.Time, out.q.data(:,2),'LineWidth', 1.2);
%hold on; plot(out.qd.Time, out.qd.data(:,2),'LineWidth', 1.2);
yline(out.qd.data(1,2),'-','');
xlabel('time [s]');
ylabel('q [rad/s]');
legend('q2', 'qd2');

subplot(2,3,3);
hold on;
grid on;
plot(out.q.Time, out.q.data(:,3),'LineWidth', 1.2);
%hold on; plot(out.qd.Time, out.qd.data(:,3),'LineWidth', 1.2);
yline(out.qd.data(1,3),'-','');
xlabel('time [s]');
ylabel('q [rad/s]');
legend('q3', 'qd3');

subplot(2,3,4);
hold on;
grid on;
plot(out.q.Time, out.tau.data(:,1),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('tau [Nm]');
legend('tau1');

subplot(2,3,5);
hold on;
grid on;
plot(out.q.Time, out.tau.data(:,2),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('tau [Nm]');
legend('tau2');

subplot(2,3,6);
hold on;
grid on;
plot(out.q.Time, out.tau.data(:,3),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('tau [Nm]');
legend('tau3');