%Use this object as a 'free' variable. This variable can be set after
% compiling by calling the compiled function with arguments. Eg:
% "myProblem_RUN" would become "myProblem_RUN(1, [1 2;3 4])". Represents a
% numeric value
%
%
%  Example:
%    >> input1 = acado.MexInput();
%
%  See also:
%    acado.MexInputVector
%    acado.MexInputMatrix
%
%  Licence:
%    This file is part of ACADO Toolkit  - (http://www.acadotoolkit.org/)
%
%    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
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
%    Date: 2010
% 
classdef MexInput < acado.Expression    
    properties(SetAccess='protected')
        type;
        counter;
    end
    
    methods
        function obj = MexInput(varargin)
            checkActiveModel;
            
            global ACADO_;
            obj.counter = ACADO_.count_mexin;
            obj.name = strcat('mexinput', num2str(ACADO_.count_mexin));
            ACADO_.count_mexin = ACADO_.count_mexin+1;   % START WITH ZERO!
            
            obj.type = 1;
             
            ACADO_.helper.addIn(obj);
            ACADO_.helper.addInstruction(obj);  
        end
        
        getInstructions(obj, cppobj, get)

    end
    
end