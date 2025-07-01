function F_att = attractive_force(x, x_goal, d, Katt)
if norm(x - x_goal) <= d
    F_att = Katt * norm(x - x_goal);
elseif norm(x - x_goal) > d
        F_att  = d * Katt * (x - x_goal)/ norm(x - x_goal) ; 
end
end


