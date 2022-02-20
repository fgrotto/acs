figure();
subplot(3,3,1);
hold on;
grid on;
plot(out.f.Time, out.f.data(:,1),'LineWidth', 1.2);
hold on; plot(out.fd.Time, out.fd.data(:,1),'LineWidth', 1.2);
yline(out.fd.data(1,1),'-','');
xlabel('time [s]');
ylabel('f');
legend('fx', 'fxd');

subplot(3,3,2);
hold on;
grid on;
plot(out.f.Time, out.f.data(:,2),'LineWidth', 1.2);
hold on; plot(out.fd.Time, out.fd.data(:,2),'LineWidth', 1.2);
yline(out.fd.data(1,2),'-','');
xlabel('time [s]');
ylabel('f');
legend('fy', 'fyd');

subplot(3,3,3);
hold on;
grid on;
plot(out.f.Time, out.f.data(:,3),'LineWidth', 1.2);
hold on; plot(out.fd.Time, out.fd.data(:,3),'LineWidth', 1.2);
%yline(out.fd.data(1,3),'-','');
xlabel('time [s]');
ylabel('f');
legend('fz', 'fzd');

subplot(3,3,4);
hold on;
grid on;
plot(out.f.Time, out.f.data(:,4),'LineWidth', 1.2);
hold on; plot(out.fd.Time, out.fd.data(:,4),'LineWidth', 1.2);
%yline(out.fd.data(1,4),'-','');
xlabel('time [s]');
ylabel('f1');
legend('f1', 'f1d');

subplot(3,3,5);
hold on;
grid on;
plot(out.f.Time, out.f.data(:,5),'LineWidth', 1.2);
hold on; plot(out.fd.Time, out.fd.data(:,5),'LineWidth', 1.2);
%yline(out.fd.data(1,5),'-','');
xlabel('time [s]');
ylabel('f2');
legend('f2', 'f2d');

subplot(3,3,6);
hold on;
grid on;
plot(out.f.Time, out.f.data(:,6),'LineWidth', 1.2);
hold on; plot(out.fd.Time, out.fd.data(:,6),'LineWidth', 1.2);
%yline(out.fd.data(1,6),'-','');
xlabel('time [s]');
ylabel('f3');
legend('f3', 'f3d');

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