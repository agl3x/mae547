function [EOM] = EOM(dh,mL,mm,IL,Im,kr,g0,Joint_type) 
    % Find jacobians & Equations of motion for a robotic arm
    % dh - Denavit-Hartenberg table - order (a, alpha, d, theta) [n x4]
    % mL - mass of each link [1xn]
    % mm - mas of each motor [1xn]
    % kr - motor ratio
    % Joint_type "P"-Prismatic, "R" Revolute
    % g0 [x y z] , input direction of gravity


    n = size(dh,1); % # of joints
    
    B = sym(zeros(n));
    c = sym(zeros(n));

    %% Make transformation Matricies 
    T0i = sym(zeros(4, 4, n));
    T    = sym(zeros(4, 4, n));
    for i = 1:n
        T(:,:,i) = homo_trans(dh(i,1),dh(i,2),dh(i,3),dh(i,4));

        if i == 1
            T0i(:,:,i) = T(:,:,i);
        else
            T0i(:,:,i) = T0i(:,:,i-1)* T(:,:,i);
        end
    end



    %% Make Jacobians
    JpL = sym(zeros(3, n, n));
    JoL = sym(zeros(3, n, n));
    JpM = sym(zeros(3, n, n));
    JoM = sym(zeros(3, n, n));

    for i = 1:n
        if i == 1
            P_prev = [0; 0; 0];
        else
            P_prev = T0i(1:3, 4, i-1);
        end
        P_Li = (P_prev + T0i(1:3, 4, i))/ 2;

        if i == 1
            P_mi = [0; 0; 0];        % joint 1 is at the base origin
        else
            P_mi = T0i(1:3, 4, i-1); % origin of frame i-1 = location of joint i
        end
        if i == 1
            z_mi = [0; 0; 1];           % motor on base link
        else
            z_mi = T0i(1:3, 3, i-1);   % z-axis of frame i-1
        end

        
        for j = 1:i
            if j == 1
                z_j1 = [0; 0; 1]; % base z-axis
                P_j1 = [0; 0; 0];  %Pj-1
            else
                z_j1 = T0i(1:3, 3, j-1); % z_j-1
                P_j1 = T0i(1:3, 4, j-1); %P_j-1
            end

            %% Link Jacobians
            if Joint_type(j) == "R" %revolute
                JpL(:,j,i) = cross(z_j1,P_Li - P_j1); % z_j-1 x (P_Li - P_j-1)
                JoL(:,j,i) = z_j1; % z_j-1
            else
                JpL(:,j,i) = z_j1; % z_j-1
                JoL(:,j,i) = [0; 0; 0];
            end

            %% Motor Angle Jacobian
            if j == i
                JoM(:,j,i) = kr(i)*z_mi; %kr * z_mi
            else
                JoM(:,j,i) = JoL(:,j,i);
            end
        end

        for j = 1:i-1
            if j == 1
                z_j1 = [0; 0; 1]; % base z-axis
                P_j1 = [0; 0; 0];  %Pj-1
            else
                z_j1 = T0i(1:3, 3, j-1); % z_j-1
                P_j1 = T0i(1:3, 4, j-1); %P_j-1
            end

            %% Motor Translation Jacobians
            if Joint_type(j) == "R" %revolute
                JpM(:,j,i) = cross(z_j1,P_mi - P_j1); % z_j-1 x (P_mi - P_j-1)
            else
                JpM(:,j,i) = z_j1; % z_j-1
            end

        end

    end
    JpL = simplify(JpL);
    JoL = simplify(JoL);
    JpM = simplify(JpM);
    JoM = simplify(JoM);

    %% Save Jacobians 
    J.JpL = JpL;
    J.JoL = JoL;
    J.JpM = JpM;
    J.JoM = JoM;

 
    %% Make B matrix
    for i=1:n
        R_0i = T0i(1:3, 1:3, i);

        if i == 1
            R_0mi = eye(3);
        else
            R_0mi = T0i(1:3, 1:3, i-1);
        end

        Jp_i  = JpL(:, :, i);   
        Jo_i  = JoL(:, :, i);   
        Jpm_i = JpM(:, :, i);   
        Jom_i = JoM(:, :, i); 

        B = B + mL(i)*(Jp_i.'*Jp_i) + Jo_i.'*R_0i*IL(i)*R_0i.'*Jo_i ...
              + mm(i)*(Jpm_i.'*Jpm_i) + Jom_i.'*R_0mi*Im(i)*R_0mi.'*Jom_i;
       
    end

    EOM.B = simplify(B);

    %% identify q
    % Make base q vector for each joint
    q   = sym('q',   [n 1]);
    for i = 1:n
        if Joint_type(i) == 'R'
            q(i) = dh(i,4);
        else
            q(i) = dh(i,3);
        end
    end

    % Make derivative qd & qdd for each variable type
    qd  = sym(zeros(n,1));
    qdd = sym(zeros(n,1));
    for i = 1:n
        varname = char(q(i));           % e.g. 't1'
        qd(i)  = sym([varname, 'd']);   % e.g. 't1d'
        qdd(i) = sym([varname, 'dd']);  % e.g. 't1dd'
    end

    EOM.Q.q = q;
    EOM.Q.qd = qd;
    EOM.Q.qdd = qdd;

    %% Make C matrix

    for k = 1:n
        for j = 1:n
            for i = 1:n
                c(k,j) = c(k,j) + 0.5*(diff(B(k,j), q(i)) + ...
                                        diff(B(k,i), q(j)) - ...
                                        diff(B(i,j), q(k))) * ...
                                        qd(i);

            end
        end
    end

    EOM.c = simplify(c);

    %% Make G Matrix

    g_vec = g0(:); % ensure column vector [3x1]
    G = sym(zeros(n,1));
    for i = 1:n
        G = G + mL(i) * (g_vec.' * JpL(:,:,i)).' ...
              + mm(i) * (g_vec.' * JpM(:,:,i)).';
    end

    EOM.G = -simplify(G);



    %% Combine Matricies into EOM
    EOM.tau = sym('tau', [n 1]);

    EOM.tau = simplify(B*qdd + c*qd + G);
end


function A = homo_trans(a, alpha, d, theta)
    c = @(x) cos(x);
    s = @(x) sin(x);

    A = [c(theta), -s(theta)*c(alpha), s(theta)*s(alpha), a*c(theta);
         s(theta), c(theta)*c(alpha), -c(theta)*s(alpha), a*s(theta)
         0,       s(alpha),           c(alpha),            d;
         0,        0,                     0,                1       ];
end