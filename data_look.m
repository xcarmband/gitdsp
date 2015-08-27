load data.txt;
subplot(4,4,1);
plot(data(:,1),data(:,2));

subplot(4,4,5);
plot(data(:,1),data(:,3));

subplot(4,4,9);
plot(data(:,1),data(:,4));

subplot(4,4,13);
plot(data(:,1),sqrt(data(:,4).^2 + data(:,3).^2 + data(:,2).^2));


subplot(4,4,2);
plot(data(:,1),data(:,8));
hold on;
plot(data(:,1),data(:,5),'r');

subplot(4,4,6);
plot(data(:,1),data(:,9));
hold on;
plot(data(:,1),data(:,6),'r');

subplot(4,4,10);
plot(data(:,1),data(:,10));
hold on;
plot(data(:,1),data(:,7),'r');

subplot(4,4,14);
plot(data(:,1),sqrt(data(:,8).^2 + data(:,9).^2 + data(:,10).^2));
hold on;
plot(data(:,1),sqrt(data(:,7).^2 + data(:,6).^2 + data(:,5).^2),'r');


v = zeros(length(data(:,5)),3);
for i = 1:length(v)
    v(i,1) = sum(data(1:i,5));
    v(i,2) = sum(data(1:i,6));
    v(i,3) = sum(data(1:i,7));
end


subplot(4,4,3);
plot(data(:,1),data(:,11));
hold on;
plot(data(:,1),data(:,14),'r');
hold on;
plot(data(:,1),v(:,1),'g');

subplot(4,4,7);
plot(data(:,1),data(:,12));
hold on;
plot(data(:,1),data(:,15),'r');
hold on;
plot(data(:,1),v(:,2),'g');


subplot(4,4,11);
plot(data(:,1),data(:,13));
hold on;
plot(data(:,1),data(:,16),'r');
hold on;
plot(data(:,1),v(:,3),'g');


subplot(4,4,15);
plot(data(:,1),sqrt(data(:,11).^2 + data(:,12).^2 + data(:,13).^2));
hold on;
plot(data(:,1),sqrt(data(:,14).^2 + data(:,15).^2 + data(:,16).^2),'r');


p = zeros(length(data(:,5)),3);
for i = 1:length(p)
    p(i,1) = sum(data(1:i,14));
    p(i,2) = sum(data(1:i,15));
    p(i,3) = sum(data(1:i,16));
end


subplot(4,4,4);
plot(data(:,1),data(:,17));
hold on;
plot(data(:,1),data(:,20),'r');
hold on;
plot(data(:,1),p(:,1),'g');

subplot(4,4,8);
plot(data(:,1),data(:,18));
hold on;
plot(data(:,1),data(:,21),'r');
hold on;
plot(data(:,1),p(:,2),'g');


subplot(4,4,12);
plot(data(:,1),data(:,19));
hold on;
plot(data(:,1),data(:,22),'r');
hold on;
plot(data(:,1),p(:,3),'g');


subplot(4,4,16);
plot3(data(:,20),data(:,21),data(:,22));
grid on
