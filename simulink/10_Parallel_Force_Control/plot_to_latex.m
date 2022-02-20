figure();
subplot(2,3,1);
hold on;
grid on;
plot(out.f.Time, out.f.data(:,1),'LineWidth', 1.2);
%hold on; plot(out.fd.Time, out.fd.data(:,1),'LineWidth', 1.2);
yline(out.fd.data(1,1),'-','');
xlabel('time [s]');
ylabel('f [force] Nm');
legend('fx', 'fxd');

subplot(2,3,2);
hold on;
grid on;
plot(out.x.Time, out.x.data(:,1),'LineWidth', 1.2);
%hold on; plot(out.xd.Time, out.xd.data(:,1),'LineWidth', 1.2);
yline(out.xd.data(1,1),'-','');
xlabel('time [s]');
ylabel('x [position] m');
legend('x', 'xd');

subplot(2,3,3);
hold on;
grid on;
plot(out.x.Time, out.x.data(:,2),'LineWidth', 1.2);
%hold on; plot(out.xd.Time, out.xd.data(:,1),'LineWidth', 1.2);
yline(out.xd.data(1,2),'-','');
xlabel('time [s]');
ylabel('y [position] m');
legend('y', 'yd');

subplot(2,3,4);
hold on;
grid on;
plot(out.f.Time, out.tau.data(:,1),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('tau [Nm]');
legend('tau1');

subplot(2,3,5);
hold on;
grid on;
plot(out.f.Time, out.tau.data(:,2),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('tau [Nm]');
legend('tau2');

subplot(2,3,6);
hold on;
grid on;
plot(out.f.Time, out.tau.data(:,3),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('tau [Nm]');
legend('tau3');