function [ q,vq,aq, t ] = path_planning(tm, dt,qstates,constraints )
%Given a time, the Denavit-Hartenberg parameter matrix, a set of Points P
%and a corresponding set of orientations R, the type of trajectory amd the 
%constraints the function gives the trajectory (in configuration space) to 
%achieve the points in the order they are given
%t_m = time to perform the maneuver from the first point to the last [s]
%dt = timestep [s]
%dh_matrix = denavit-hartenberg parameter matrix
%P = list of points in the required motion
%R = list of orientations in required motion
%qstate = initial configuration of joints
%type = type of motion, 'Cubic', 'Quintic' or 'LSPB'
% constraints = 2x6 set of constraints on velocities and accelerations
tarray = 0:dt:tm(end);
q = zeros(size(tarray,2),6);
size(q)
vq = zeros(size(tarray,2),6);
aq = zeros(size(tarray,2),6);
%Always assumed that start and end velocity of path are at 0 velocity
if size(qstates,1) ==2
    for ii = 1:1:6
        vmax = constraints(1,ii);
        if vmax>2*abs(qstates(2,ii)-qstates(1,ii))/tm(end)
            vmax = 2*(qstates(2,ii)-qstates(1,ii))/tm(end);
        end
        alpha =vmax^2/(vmax*tm+qstates(1,ii)-qstates(2,ii));
        tb = (vmax*tm+qstates(1,ii)-qstates(2,ii))/vmax;
        tbindex = round(tb/dt);
        tbround = tbindex*dt;
        q(1:tbindex,ii) = qstates(1,ii)+0.5*alpha*tarray(1:tbindex).^2;
        vq(1:tbindex,ii) = alpha*tarray(1:tbindex);
        aq(1:tbindex,ii) = alpha;
        if tbindex~=(size(q,1)-tbindex)
            q(tbindex+1:end-tbindex,ii)= qstates(1,ii)+alpha*tbround*(tarray(tbindex+1:end-tbindex)-tbround/2);
            vq(tbindex+1:end-tbindex,ii) = vmax;
            aq(tbindex+1:end-tbindex,ii) = 0;
        end
        q(end-tbindex+1:end,ii) = qstates(2,ii)-0.5*alpha*(tm-tarray((end-tbindex+1):end)).^2;
        vq(end-tbindex+1:end,ii) = alpha*(tm-tarray((end-tbindex+1):end));
        aq(end-tbindex+1:end,ii) = -alpha;
    end
else
    if size(tm) >1
        td = tm; %Tm is the times for each of the  data points
    else
        td = [0;tm/(size(qstates,1)-1)*cumsum(ones((size(qstates,1)-1),1))]; % Or tm is only the end time, each point takes equal amount of time
    end
    for ii = 1:1:1
        alpha = constraints(2,ii);%maximum acceleration
        %Calculate velocities
        vec = (qstates(2:end,ii)-qstates(1:end-1,ii))./(td(2:end)-td(1:end-1));
        %Calculate blend times
        t1 = td(2)-sqrt(td(2)^2-2*(qstates(2,ii)-qstates(1,ii))/alpha);
        tend =  (td(end)-td(end-1))-sqrt((td(end)-td(end-1))^2-2*abs(qstates(end,ii)-qstates(end-1,ii))/alpha);
        t1index= round(t1/dt);
        tendindex =round(tend/dt);
        vec(1) = (qstates(2,ii)-qstates(1,ii))/(td(2)-0.5*t1);  %Fix start and end segment velocities
        vec(end) = (qstates(end,ii)-qstates(end-1,ii))/(td(2)-0.5*tend);
        tk = abs(vec(2:end)-vec(1:end-1))/alpha; %transition times

        %Calculate position as function of time
        for jj = 1:1:2*size(qstates,1)-1 %loop over every point (qstates size) and every transition (qstates-1)
            if jj== 1% Beginnning 
                q(1:t1index,ii) =qstates(1,ii)+0.5*alpha*sign(vec(1))*tarray(1:t1index).^2;
                vq(1:t1index,ii) = alpha*sign(vec(1))*tarray(1:t1index);
                aq(1:t1index,ii) = alpha*sign(vec(1));
            elseif jj==2*size(qstates,1)-1 %End
                q(end-tendindex+1:end,ii) =q(end-tendindex,ii)+vq(end-tendindex,ii)*(tarray(end-tendindex+1:end)-tarray(end-tendindex))+0.5*alpha*sign(-vec(end))*(tarray(end-tendindex+1:end)-tarray(end-tendindex)).^2;
                vq(end-tendindex+1:end,ii) = vq(end-tendindex,ii)+alpha*sign(-vec(end))*(tarray(end-tendindex+1:end)-tarray(end-tendindex));
                aq(end-tendindex+1:end,ii) = alpha*sign(-vec(end));
            elseif mod(jj,2)==1 %Uneven integers are parabolic blending times
                %Indices are relevant time for datapoint - and + half the
                %transition time
                startindex = round((td((jj+1)/2)-tk((jj-1)/2)/2)/dt)+1;
                endindex =   round((td((jj+1)/2)+tk((jj-1)/2)/2)/dt);
                q(startindex:endindex,ii) =q(startindex-1)+vec((jj-1)/2)*(tarray(startindex:endindex)-tarray(startindex-1))+0.5*alpha*sign(vec((jj+1)/2)-vec((jj-1)/2))*(tarray(startindex:endindex)-tarray(startindex-1)).^2;
                vq(startindex:endindex,ii) =vec((jj-1)/2)+alpha*sign(vec((jj+1)/2)-vec((jj-1)/2))*(tarray(startindex:endindex)-tarray(startindex-1));
                aq(startindex:endindex,ii) = alpha*sign(vec((jj+1)/2)-vec((jj-1)/2));
            else %Everything else are linear transition
                %Indices are times between data points
                if jj == 2
                    startindex = t1index+1;
                    endindex =round((td((jj)/2+1)-tk((jj)/2)/2)/dt);
                elseif jj == 2*size(qstates,1)-2
                    startindex = round((td((jj)/2)+tk((jj)/2-1)/2)/dt)+1;
                    endindex = size(tarray,2)-tendindex;
                else
                    startindex = round((td((jj)/2)+tk((jj)/2-1)/2)/dt)+1;
                    endindex =round((td((jj)/2+1)-tk((jj)/2)/2)/dt);
                end
                q(startindex:endindex,ii) =q(startindex-1)+vec((jj)/2)*(tarray(startindex:endindex)-tarray(startindex-1));
                vq(startindex:endindex,ii) =vec((jj)/2);
                aq(startindex:endindex,ii) = 0;
            end
        end
    end
end
t= tarray;

end


