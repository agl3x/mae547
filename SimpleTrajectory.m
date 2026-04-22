function x_d = SimpleTrajectory(t_i, t_f, x_i, x_f, degree)


if degree == 3 || degree == 5   

    if length(x_i) ~= length(x_f)
        disp("Initial and final dimensions aren't the same.")
        return
    end

    dimension = length(x_i);
    
    A = sym('A',[dimension degree+1]);
    P = sym('P', [dimension 1]);
    dP = sym('dP', [dimension 1]);
    
    E = {};
    dE = {};

    P_sol = sym('P_sol', [dimension 1]);
    
    syms t

    if degree == 3
        for i = 1:dimension
            P(i) = A(i, 1);
            dP(i) = 0;
        
            for j=2:degree+1
                P(i) = P(i) + A(i, j)*(t)^(j-1);
                dP(i) = diff(P(i), t);
            end
        
            E{i} = [subs(P(i), t, t_i) == x_i(i), subs(P(i), t, t_f) == x_f(i)];
            dE{i} = [subs(dP(i), t, t_i) == 0, subs(dP(i), t, t_f) == 0];
        
            sol = solve([E{i}, dE{i}], A(i, :));
        
            A_sol = cell2mat(struct2cell(sol))';
            P_sol(i) = subs(P(i), A(i, :), A_sol);    
        end

    

    else
        d2P = sym('d2P', [dimension 1]);
        d2E = {};

        for i = 1:dimension
            P(i) = A(i, 1);
            dP(i) = 0;
            d2P(i) = 0;
        
            for j=2:degree+1
                P(i) = P(i) + A(i, j)*(t)^(j-1);
                dP(i) = diff(P(i), t);
                d2P(i) = diff(P(i), t, 2);
            end
        
            E{i} = [subs(P(i), t, t_i) == x_i(i), subs(P(i), t, t_f) == x_f(i)];
            dE{i} = [subs(dP(i), t, t_i) == 0, subs(dP(i), t, t_f) == 0];
            d2E{i} = [subs(d2P(i), t, t_i) == 0, subs(d2P(i), t, t_f) == 0];
        
            sol = solve([E{i}, dE{i}, d2E{i}], A(i, :));
        
            A_sol = cell2mat(struct2cell(sol))';
            P_sol(i) = subs(P(i), A(i, :), A_sol);    
        end
    end

x_d = P_sol;

else
    disp("Degree must be 3 or 5")
end

end










