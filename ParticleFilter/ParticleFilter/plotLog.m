a = importdata('log.txt', ' ', 0);
hold all
plot(a(2:end,1),-a(2:end,2));
hold off