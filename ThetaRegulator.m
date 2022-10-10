function theta = ThetaRegulator(thi)

while(abs(thi)>pi)
    if(thi>0)
        thi = thi - 2*pi;
    else
        thi = thi + 2*pi;
    end
end
theta = thi;
end
