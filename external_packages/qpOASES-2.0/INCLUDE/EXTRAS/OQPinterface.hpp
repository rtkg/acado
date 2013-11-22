/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2009 by Hans Joachim Ferreau et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file INCLUDE/EXTRAS/OQPinterface.hpp
 *	\author Hans Joachim Ferreau
 *	\version 2.0
 *	\date 2008-2009
 *
 *	Declaration of an interface comprising several utility functions
 *	for solving test problems from the Online QP Benchmark Collection
 *	(see http://homes.esat.kuleuven.be/~optec/software/onlineQP/).
 */


#ifndef QPOASES_OQPINTERFACE_HPP
#define QPOASES_OQPINTERFACE_HPP


#include <Utils.hpp>


#ifndef __DSPACE__
namespace qpOASES
{
#endif

/** Reads dimensions of an Online QP Benchmark problem from file.
 * \return SUCCESSFUL_RETURN \n
		   RET_UNABLE_TO_READ_FILE \n
		   RET_FILEDATA_INCONSISTENT */
returnValue readOQPdimensions(	const char* path,	/**< Full path of the data files (without trailing slash!). */
								int& nQP,			/**< Output: Number of QPs. */
								int& nV,			/**< Output: Number of variables. */
								int& nC,			/**< Output: Number of constraints. */
								int& nEC			/**< Output: Number of equality constraints. */
								);

/** Reads data of an Online QP Benchmark problem from file.
 *  This function allocates the required memory for all data; after successfully calling it,
 *  you have to free this memory yourself!
 * \return SUCCESSFUL_RETURN \n
		   RET_INVALID_ARGUMENTS \n
		   RET_UNABLE_TO_READ_FILE \n
		   RET_FILEDATA_INCONSISTENT */
returnValue readOQPdata(	const char* path,	/**< Full path of the data files (without trailing slash!). */
							int& nQP,			/**< Output: Number of QPs. */
							int& nV,			/**< Output: Number of variables. */
							int& nC,			/**< Output: Number of constraints. */
							int& nEC,			/**< Output: Number of equality constraints. */
							double** H,		 	/**< Output: Hessian matrix. */
							double** g,		 	/**< Output: Sequence of gradient vectors. */
							double** A,		 	/**< Output: Constraint matrix. */
							double** lb,		/**< Output: Sequence of lower bound vectors (on variables). */
							double** ub,		/**< Output: Sequence of upper bound vectors (on variables). */
							double** lbA,		/**< Output: Sequence of lower constraints' bound vectors. */
							double** ubA,		/**< Output: Sequence of upper constraints' bound vectors. */
							double** xOpt,		/**< Output: Sequence of primal solution vectors
												 *           (not read if a null pointer is passed). */
							double** yOpt,		/**< Output: Sequence of dual solution vectors
												 *           (not read if a null pointer is passed). */
							double** objOpt		/**< Output: Sequence of optimal objective function values
												 *           (not read if a null pointer is passed). */
							);


/** Solves an Online QP Benchmark problem as specified by the arguments.
 *  The maximum deviations from the given optimal solution as well as the
 *  maximum CPU time to solve each QP are determined.
 * \return SUCCESSFUL_RETURN \n
 		   RET_BENCHMARK_ABORTED */
returnValue solveOQPbenchmark(	int nQP,					/**< Number of QPs. */
								int nV,						/**< Number of variables. */
								int nC,						/**< Number of constraints. */
								int nEC,					/**< Number of equality constraints. */
								const double* const H,		/**< Hessian matrix. */
								const double* const g,		/**< Sequence of gradient vectors. */
								const double* const A,		/**< Constraint matrix. */
								const double* const lb,		/**< Sequence of lower bound vectors (on variables). */
								const double* const ub,		/**< Sequence of upper bound vectors (on variables). */
								const double* const lbA,	/**< Sequence of lower constraints' bound vectors. */
								const double* const ubA,	/**< Sequence of upper constraints' bound vectors. */
								const double* const xOpt,	/**< Sequence of primal solution vectors. */
								const double* const yOpt,	/**< Sequence of dual solution vectors. */
								const double* const objOpt,	/**< Sequence of optimal objective function values. */
								int& nWSR, 					/**< Input: Maximum number of working set recalculations; \n
																 Output: Maximum number of performed working set recalculations. */
								double& maxCPUtime,			/**< Maximum CPU time required for solving each QP. */
								double& maxPrimalDeviation,	/**< Maximum deviation of primal solution vector. */
								double& maxDualDeviation,	/**< Maximum deviation of dual solution vector. */
								double& maxObjDeviation		/**< Maximum deviation of optimal objective function value. */
								);

/** Solves an Online QP Benchmark problem (without constraints) as specified
 *  by the arguments. The maximum deviations from the given optimal solution
 *  as well as the maximum CPU time to solve each QP are determined.
 * \return SUCCESSFUL_RETURN \n
 		   RET_BENCHMARK_ABORTED */
returnValue solveOQPbenchmark(	int nQP,					/**< Number of QPs. */
								int nV,						/**< Number of variables. */
								const double* const H,		/**< Hessian matrix. */
								const double* const g,		/**< Sequence of gradient vectors. */
								const double* const lb,		/**< Sequence of lower bound vectors (on variables). */
								const double* const ub,		/**< Sequence of upper bound vectors (on variables). */
								const double* const xOpt,	/**< Sequence of primal solution vectors. */
								const double* const yOpt,	/**< Sequence of dual solution vectors. */
								const double* const objOpt,	/**< Sequence of optimal objective function values. */
								int& nWSR, 					/**< Input: Maximum number of working set recalculations; \n
																 Output: Maximum number of performed working set recalculations. */
								double& maxCPUtime,			/**< Maximum CPU time required for solving each QP. */
								double& maxPrimalDeviation,	/**< Maximum deviation of primal solution vector. */
								double& maxDualDeviation,	/**< Maximum deviation of dual solution vector. */
								double& maxObjDeviation		/**< Maximum deviation of optimal objective function value. */
								);


/** Runs an Online QP Benchmark problem and determines the maximum
 *  deviations from the given optimal solution as well as the
 *  maximum CPU time to solve each QP.
 * \return SUCCESSFUL_RETURN \n
		   RET_UNABLE_TO_READ_BENCHMARK \n
 		   RET_BENCHMARK_ABORTED */
returnValue runOQPbenchmark(	const char* path,			/**< Full path of the benchmark files (without trailing slash!). */
								int& nWSR, 					/**< Input: Maximum number of working set recalculations; \n
																 Output: Maximum number of performed working set recalculations. */
								double& maxCPUtime,			/**< Maximum CPU time required for solving each QP. */
								double& maxPrimalDeviation,	/**< Maximum deviation of primal solution vector. */
								double& maxDualDeviation,	/**< Maximum deviation of dual solution vector. */
								double& maxObjDeviation		/**< Maximum deviation of optimal objective function value. */
								);

#ifndef __DSPACE__
} /* qpOASES */
#endif

#endif	/* QPOASES_OQPINTERFACE_HPP */


/*
 *	end of file
 */
