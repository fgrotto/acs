figure();
subplot(3,3,1);
hold on;
grid on;
plot(out.f.Time, out.f.data(:,1),'LineWidth', 1.2);
hold on; plot(out.fd.Time, out.fd.data(:,1),'LineWidth', 1.2);
yline(out.fd.data(1,1),'-','');
xlabel('time [s]');
ylabel('force [N]');
legend('fx', 'fxd');

subplot(3,3,2);
hold on;
grid on;
plot(out.f.Time, out.f.data(:,2),'LineWidth', 1.2);
hold on; plot(out.fd.Time, out.fd.data(:,2),'LineWidth', 1.2);
yline(out.fd.data(1,2),'-','');
xlabel('time [s]');
ylabel('force [N]');
legend('fy', 'fyd');

subplot(3,3,3);
hold on;
grid on;
plot(out.f.Time, out.f.data(:,3),'LineWidth', 1.2);
hold on; plot(out.fd.Time, out.fd.data(:,3),'LineWidth', 1.2);
%yline(out.fd.data(1,3),'-','');
xlabel('time [s]');
ylabel('force [N]');
legend('fz', 'fzd');

subplot(3,3,4);
hold on;
grid on;
plot(out.x.Time, out.x.data(:,1),'LineWidth', 1.2);
hold on; plot(out.xd.Time, out.xd.data(:,1),'LineWidth', 1.2);
%yline(out.fd.data(1,4),'-','');
xlabel('time [s]');
ylabel('x [m]');
legend('x', 'xd');

subplot(3,3,5);
hold on;
grid on;
plot(out.x.Time, out.x.data(:,2),'LineWidth', 1.2);
hold on; plot(out.xd.Time, out.xd.data(:,2),'LineWidth', 1.2);
%yline(out.fd.data(1,5),'-','');
xlabel('time [s]');
ylabel('y [m]');
legend('y', 'yd');

subplot(3,3,6);
hold on;
grid on;
plot(out.x.Time, out.x.data(:,3),'LineWidth', 1.2);
hold on; plot(out.xd.Time, out.xd.data(:,3),'LineWidth', 1.2);
%yline(out.fd.data(1,6),'-','');
xlabel('time [s]');
ylabel('z [m]');
legend('z', 'zd');

subplot(3,3,7);
hold on;
grid on;
plot(out.f.Time, out.tau.data(:,1),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('tau [Nm]');
legend('tau1');

subplot(3,3,8);
hold on;
grid on;
plot(out.f.Time, out.tau.data(:,2),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('tau [Nm]');
legend('tau2');

subplot(3,3,9);
hold on;
grid on;
plot(out.f.Time, out.tau.data(:,3),'LineWidth', 1.2);
xlabel('time [s]');
ylabel('tau [Nm]');
legend('tau3');