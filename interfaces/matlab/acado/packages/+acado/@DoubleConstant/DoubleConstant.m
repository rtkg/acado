%Constant
%
%  Usage:
%    >> DoubleConstant(value);
%
%  Parameters:
%    name 	   A numeric [NUMERIC or acado.MexInput]
%
%  Example:
%    >> acado.DoubleConstant(1.5);
%    >> acado.DoubleConstant(acado.MexInput());
%
%  Licence:
%    This file is part of ACADO Toolkit  - (http://www.acadotoolkit.org/)
%
%    ACADO Toolkit -- A Toolkit for Automatic Disturbance and Dynamic Optimization.
%    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
%    Developed within the Optimization in Engineering Center (OPTEC) under
%    supervision of Moritz Diehl. All rights reserved.
%
%    ACADO Toolkit is free software; you can redistribute it and/or
%    modify it under the terms of the GNU Lesser General Public
%    License as published by the Free Software Foundation; either
%    version 3 of the License, or (at your option) any later version.
%
%    ACADO Toolkit is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%    Lesser General Public License for more details.
%
%    You should have received a copy of the GNU Lesser General Public
%    License along with ACADO Toolkit; if not, write to the Free Software
%    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%
%    Author: David Ariens
%    Date: 2009-2010
% 
classdef DoubleConstant < acado.Operator    
    properties(SetAccess='private')
        val;
    end
    
    methods
        function obj = DoubleConstant(val)
            global ACADO_;
            
            if (isa(val, 'numeric'))
                ACADO_.count_double = ACADO_.count_double+1;
                                
                obj.val = val;
                obj.name = strcat('acadoconstant', num2str(ACADO_.count_double));
                
                ACADO_.helper.addInstruction(obj);
                
            elseif (isa(val, 'acado.MexInput'))
                
                if (val.type ~= 1)
                    error('MexInput should be in this case a numeric value, not a vector or matrix.'); 
                end

                obj.name = val.name;
                
            else
                error('DoubleConstant expects a numeric value or a acado.MexInput'); 
            end
        end
       
        
        getInstructions(obj, cppobj, get)
        
        
        function s = toString(obj)
            % toString is used in epxressions (eg 2 + x -> DoubleConstant +
            % DifferentialState)
            s = obj.name; 
            
        end
    end
end

