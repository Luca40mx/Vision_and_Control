function plotRef(R, T, label)
    % [RT] brings a transformation from refA system to ref systemB
    %
    pose=T;
    % T represents the origin of the starting ref (i.e., refA) according to
    % the ending ref (i.e., refB)
    plot3(pose(1), pose(2), pose(3),'-.','Color','b','MarkerSize',50);   
    
    %Columns of rotation R are the three axis of the starting ref (i.e,
    %refA) according to the endig ref (i.e., refB)
    hold on;
    f1 = quiver3(pose(1), pose(2), pose(3),R(1,1),R(2,1),R(3,1),50,'Color','r','DisplayName','x');
    f2 = quiver3(pose(1), pose(2), pose(3),R(1,2),R(2,2),R(3,2),50,'Color','g','DisplayName','y');
    f3 = quiver3(pose(1), pose(2), pose(3),R(1,3),R(2,3),R(3,3),50,'Color','b','DisplayName','z');
    text(pose(1), pose(2), pose(3),label);
end

