function diff = SpecialModify(SenseData,Zest,i)
diff = SenseData(i,2:3)'-Zest;
if(Zest(2)*SenseData(i,3)<0 && (abs(Zest(2))+abs(SenseData(i,3)))>pi)
   disp('Special modify\n')
   absDifference = pi - abs(SenseData(i,3)) + pi - abs(Zest(2));
   if(Zest(2)<0)
      diff(2) = -1*absDifference;
   else
      diff(2) = absDifference;
   end
 
end

end