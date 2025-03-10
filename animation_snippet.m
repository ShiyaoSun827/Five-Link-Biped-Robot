%% Animation of the simulation results
Axis=[-1 4 0 2];
axis equal;
Time=text(1,1.8,['time= ',num2str(t1(1)),' secs']);
axis(Axis);
q=q0;

q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);
xdata=0;
ydata=0;
l=[l1 l2 l2 l1 l3];
Q=[q1 q1+q2 q1+q2+q3 q1+q2+q3+q4 q1+q2+q5];
for j=1:4
    xdata=[xdata xdata(end)+l(j)*cos(Q(j))];
    ydata=[ydata ydata(end)+l(j)*sin(Q(j))];
end
xdata=[xdata xdata(3)+l(5)*cos(Q(5))];
ydata=[ydata ydata(3)+l(5)*sin(Q(5))];

link1=line([xdata(1) xdata(2)],[ydata(1) ydata(2)],'color','red','linewidth',2);
link2=line([xdata(2) xdata(3)],[ydata(2) ydata(3)],'color','red','linewidth',2);
link3=line([xdata(3) xdata(4)],[ydata(3) ydata(4)],'linewidth',2);
link4=line([xdata(4) xdata(5)],[ydata(4) ydata(5)],'linewidth',2);
link5=line([xdata(3) xdata(6)],[ydata(3) ydata(6)],'linewidth',2);

fprintf('\n Animation is ready...\n')
ref=0; % This variable keeps track of the position of the stance foot accross multiple steps

animation_slowdown_factor=16; % >1 means slow down
for k=2:length(t1)
    t0=clock;
    drawnow;
    q=x1(k,1:5)';
    q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);
    Q=[q1 q1+q2 q1+q2+q3 q1+q2+q3+q4 q1+q2+q5];
    xdata=ref;
    ydata=0;
    for j=1:4
        xdata=[xdata xdata(end)+l(j)*cos(Q(j))];
        ydata=[ydata ydata(end)+l(j)*sin(Q(j))];
    end
    xdata=[xdata xdata(3)+l(5)*cos(Q(5))];
    ydata=[ydata ydata(3)+l(5)*sin(Q(5))];
    set(link1,'xdata',[xdata(1) xdata(2)],'ydata',[ydata(1) ydata(2)]);
    set(link2,'xdata',[xdata(2) xdata(3)],'ydata',[ydata(2) ydata(3)]);
    set(link3,'xdata',[xdata(3) xdata(4)],'ydata',[ydata(3) ydata(4)]);
    set(link4,'xdata',[xdata(4) xdata(5)],'ydata',[ydata(4) ydata(5)]);
    set(link5,'xdata',[xdata(3) xdata(6)],'ydata',[ydata(3) ydata(6)]);
    set(Time,'String',['time= ',num2str(round(t1(k),1)),' secs']);
    axis(Axis);
    while etime(clock,t0)<animation_slowdown_factor*(t1(k)-t1(k-1))
    end
end

ref=xdata(5);

for k=2:length(t2)
    t0=clock;
    drawnow;
    q=x2(k,1:5)';
    q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);
    Q=[q1 q1+q2 q1+q2+q3 q1+q2+q3+q4 q1+q2+q5];
    xdata=ref;
    ydata=0;
    for j=1:4
        xdata=[xdata xdata(end)+l(j)*cos(Q(j))];
        ydata=[ydata ydata(end)+l(j)*sin(Q(j))];
    end
    xdata=[xdata xdata(3)+l(5)*cos(Q(5))];
    ydata=[ydata ydata(3)+l(5)*sin(Q(5))];
    set(link1,'xdata',[xdata(1) xdata(2)],'ydata',[ydata(1) ydata(2)]);
    set(link2,'xdata',[xdata(2) xdata(3)],'ydata',[ydata(2) ydata(3)]);
    set(link3,'xdata',[xdata(3) xdata(4)],'ydata',[ydata(3) ydata(4)]);
    set(link4,'xdata',[xdata(4) xdata(5)],'ydata',[ydata(4) ydata(5)]);
    set(link5,'xdata',[xdata(3) xdata(6)],'ydata',[ydata(3) ydata(6)]);
    set(Time,'String',['time= ',num2str(round(t2(k),1)),' secs']);
    axis(Axis);
    while etime(clock,t0)<animation_slowdown_factor*(t2(k)-t2(k-1))
    end
end
