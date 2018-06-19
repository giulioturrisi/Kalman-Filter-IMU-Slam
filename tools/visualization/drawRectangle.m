function drawRectangleInMap(map_, row_, col_, color_code_)
  hold on;
  rectangle("Position", [col_-1 rows(map_)-row_ 1 1], "FaceColor", color_code_);
  hold off;
endfunction

