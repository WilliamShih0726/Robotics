function YN = exceed_range(joint)
YN = ~[-150<=joint(1) && joint(1)<=150 
      -30<=joint(2) && joint(2)<=100 
      -120<=joint(3) && joint(3)<=0 
      -110<=joint(4) && joint(4)<=110 
      -180<=joint(5) && joint(5)<=180 
      -180<=joint(6) && joint(6)<=180];
end

