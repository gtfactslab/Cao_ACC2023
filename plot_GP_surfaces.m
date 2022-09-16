hold on
res = 25;
global mask;
surf(full_x1(1:res:end, 1:res:end),full_x2(1:res:end, 1:res:end),full_mean(1:res:end, 1:res:end),'FaceColor','b')
surf(full_x1(1:res:end, 1:res:end),full_x2(1:res:end, 1:res:end),full_actual(1:res:end, 1:res:end),'FaceColor','m')
surf(full_x1(1:res:end, 1:res:end),full_x2(1:res:end, 1:res:end),full_upper(1:res:end, 1:res:end), 'FaceColor','k', 'FaceAlpha', 0.2, 'EdgeAlpha', 0.2)
surf(full_x1(1:res:end, 1:res:end),full_x2(1:res:end, 1:res:end),full_lower(1:res:end, 1:res:end), 'FaceColor','k', 'FaceAlpha', 0.6, 'EdgeAlpha', 0.6)
if exist('mask','var') & any(any(mask))
    roi_lower = full_lower;
    roi_upper = full_upper;
    roi_x1 = full_x1;
    roi_x2 = full_x2;
    roi_x1(~mask) = NaN;
    roi_x2(~mask) = NaN;
    roi_upper(~mask) = NaN;
    roi_lower(~mask) = NaN;
    if ~complete
        surf(roi_x1(1:res:end, 1:res:end),roi_x2(1:res:end, 1:res:end),roi_lower(1:res:end, 1:res:end), 'FaceColor', 'r')
        surf(roi_x1(1:res:end, 1:res:end),roi_x2(1:res:end, 1:res:end),roi_upper(1:res:end, 1:res:end), 'FaceColor', 'r')
    else
        surf(roi_x1(1:res:end, 1:res:end),roi_x2(1:res:end, 1:res:end),roi_lower(1:res:end, 1:res:end), 'FaceColor', 'g')
        surf(roi_x1(1:res:end, 1:res:end),roi_x2(1:res:end, 1:res:end),roi_upper(1:res:end, 1:res:end), 'FaceColor', 'g')
    end
end
%view(30, 65)
xlabel('x_1')
ylabel('x_2')
zlabel('gamma')
hold off