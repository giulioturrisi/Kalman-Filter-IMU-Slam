## Copyright (C) 2003-2011 David Legland <david.legland@grignon.inra.fr>
## Copyright (C) 2016 Adapted to Octave by Juan Pablo Carbajal <carbajal@ifi.uzh.ch>
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
##     1 Redistributions of source code must retain the above copyright notice,
##       this list of conditions and the following disclaimer.
##     2 Redistributions in binary form must reproduce the above copyright
##       notice, this list of conditions and the following disclaimer in the
##       documentation and/or other materials provided with the distribution.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ''AS IS''
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR
## ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
## DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
## SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
## CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
## OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## The views and conclusions contained in the software and documentation are
## those of the authors and should not be interpreted as representing official
## policies, either expressed or implied, of the copyright holders.

## -*- texinfo -*-
## @deftypefn {Function File} {@var{h} =} fillPolygon (@var{p})
## @deftypefnx {Function File} {@var{h} =} fillPolygon (@var{px},@var{py})
## Fill a polygon specified by a list of points
##
##   @var{p} is a single [N*2] array.
##   With one input argument, it fills the interior of the polygon specified
##   by @var{p}. The boundary of the polygon is not drawn, use 'drawPolygon'
##   to do it.
##
##   If @var{p} contains NaN-couples, each portion between the [NaN;NaN] will
##   be filled separately.
##
##   Two input arguments specify coordinates of the polygon in separate arrays.
##
##   Also returns a handle to the created patch.
##
## @seealso{polygons2d, drawCurve, drawPolygon}
## @end deftypefn

function h = fillPolygon (varargin)

  # check input
  if isempty (varargin)
      error ('need to specify a polygon');
  end

  # case of a set of polygons stored in a cell array
  var = varargin{1};
  if iscell (var)
      N = length (var);
      h = zeros (N, 1);
      for i = 1:N
          # check for empty polygons
          if ~isempty (var{i})
              h(i) = fillPolygon (var{i}, varargin{2:end});
          end
      end

      return;
  end

  # Extract coordinates of polygon vertices
  if size (var, 2) > 1
      # first argument is a polygon array
      px = var(:, 1);
      py = var(:, 2);
      varargin(1) = [];
  else
      # arguments 1 and 2 correspond to x and y coordinate respectively
      if length (varargin) < 2
          error ('should specify either a N*2 array, or 2 N*1 vectors');
      end

      px = varargin{1};
      py = varargin{2};
      varargin(1:2) = [];
  end

  # Find position of breaks, and copies first point of each loop at the
  # end
  inds = find (isnan (px(:)));
  i1   = [inds ; length(px)+1];
  i0   = [1 ; inds+1];

  px(i1, :) = px(i0, :);
  py(i1, :) = py(i0, :);

  # set default line format
  if isempty (varargin)
      varargin = {'b'};
  end

  # fill the polygon with desired style
  h = fill (px, py, varargin{:});%, 'lineStyle', 'none');

endfunction

