function gravity_centre = find_gravity_centre(aerofoil_geometry, skin_thickness, D_section_thickness, spar_thickness_front, spar_thickness_rear, spar_location_front, spar_location_rear, spar_height)

panel_num = size(aerofoil_geometry, 1) - 1; % Number of elemental panels the wing skin is divided into

for i = 1:panel_num
    panel_length = norm(aerofoil_geometry(i, :) - aerofoil_geometry(i + 1, :)); % Elemental length
    panel_centre = (aerofoil_geometry(i, :) + aerofoil_geometry(i + 1, :)) / 2; % Elemental center coordinate
    if panel_centre(1) < spar_location_front % Determine if the element belongs to the D-section, wing box, or trailing edge based on its position
        panel_thickness = D_section_thickness; % Elemental thickness
    elseif panel_centre(1) > spar_location_rear
        panel_thickness = skin_thickness;
    else
        panel_thickness = skin_thickness;
    end
    weight_skin(i, :) = [panel_centre, panel_length * panel_thickness]; % Elemental (center) position and elemental mass (actually, the cross-sectional area)
end
weight_spar = [spar_location_front, 0, spar_height * spar_thickness_front;
               spar_location_rear, 0, spar_height * spar_thickness_rear];
weight = [weight_skin; weight_spar];

gravity_centre(1) = sum(weight(:, 1) .* weight(:, 3)) / sum(weight(:, 3));
gravity_centre(2) = sum(weight(:, 2) .* weight(:, 3)) / sum(weight(:, 3));
end
