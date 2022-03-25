package mtsp

import (
	"fmt"
	"git.solver4all.com/azaryc2s/gorobi/gurobi"
	"git.solver4all.com/azaryc2s/tsp"
	"log"
	"math"
)

/*var (
	logInfo *log.Logger
	logWarn *log.Logger
	logErr *log.Logger
)*/
var (
	CutsBendersCount int
	CutsSECCount     int
)

/* Given an integer-feasible solution 'sol', find the smallest sub-tour.  Result is returned in 'tour', and length is returned in 'tourlenP'. */

func Findsubtour(edges [][]int) (result []int, isInvalid bool) {
	n := len(edges)
	seen := make([]bool, n)
	tour := make([]int, n)

	start := 0
	bestlen := n + 1
	bestind := -1
	i := 0
	node := 0
	hasNoDepot := false
	isInvalid = false
	var depotTour []int
	for start < n {
		if start > 0 && seen[0] {
			hasNoDepot = true
		}
		for node = 0; node < n; node++ {
			if !seen[node] {
				break
			}
		}
		if node == n {
			break
		}
		isConnected := false
		for leng := 0; leng < n; leng++ {
			tour[start+leng] = node
			seen[node] = true
			for i = 0; i < n; i++ {
				if edges[node][i] == 1 && !seen[i] {
					node = i
					isConnected = true
					break
				}
			}
			if i == n {
				leng++
				//with nonbinary Y-variables, its possible, that this is not an integer subtour, but if a->b->...->z edges being binary,
				//it will still cut it off, even if z->a is not part of the solution, because a and z still have to be connected to somewhere else
				if isConnected && leng < bestlen && leng > 1 {
					if hasNoDepot {
						isInvalid = true
						bestlen = leng
						bestind = start
					} else {
						depotTour = tour[start : start+leng]
					}
				}
				start += leng
				break
			}
		}
	}
	if isInvalid {
		return tour[bestind : bestind+bestlen], true
	}
	return depotTour, false

}

/* Subtour elimination callback.  Whenever a feasible solution is found, find the shortest subtour and then add the subtour elimination constraint if that tour doesn't visit every node. */

func LPCallbackMTSP(model *gurobi.Model, cbdata gurobi.CPVoid, where int32, usrdata interface{}) int32 {
	modelData := usrdata.(*MTSPModel)
	N := modelData.N
	M := modelData.M

	if where == gurobi.CB_MIPSOL {
		sol, err := gurobi.CbGetDblArray(cbdata, where, gurobi.CB_MIPSOL_SOL, modelData.VarCount)
		if err != nil {
			//log.Println(err)
			Log(1, err.Error())
		}
		solA := ExtractEdgeMatrix(sol, N, M, modelData.YStart, modelData.GMastermodel)
		nodeAss := ExtractNodeMatrix(sol, N, M, modelData.XStart)
		var subtours [][]int
		//for each machine, check if there are invalid subtours that do not contain the depot and save those
		//to derive SECs from them later
		for i := 0; i < M; i++ {
			//log.Printf("Looking for subtours in edgeMatrix %d : \n%v\n",i,solA[i])
			Log(4, "Looking for subtours in edgeMatrix %d : \n%v\n", i, solA[i])
			tour, isTourInvalid := Findsubtour(solA[i])
			nodeCount := 0
			for j := 0; j < len(nodeAss[i]); j++ {
				if nodeAss[i][j] == 1 {
					nodeCount++
				}
			}
			if isTourInvalid {
				subtours = append(subtours, tour)
			}
		}
		if len(subtours) > 0 {
			//Add the SECs
			secInd, secVal, op, rhs := getSECs(modelData, subtours, N, M, modelData.YStart)
			for i := 0; i < len(secInd); i++ {
				err = gurobi.CbLazy(cbdata, len(secInd[i]), secInd[i], secVal[i], op, rhs[i])
				if err != nil {
					//log.Println(err)
					Log(1, err.Error())
				}
			}
		}
	}

	return 0
}

func BCHCallbackMTSP(model *gurobi.Model, cbdata gurobi.CPVoid, where int32, usrdata interface{}) int32 {
	modelData := usrdata.(*MTSPModel)
	N := modelData.N
	M := modelData.M

	if where == gurobi.CB_MIPSOL {
		sol, err := gurobi.CbGetDblArray(cbdata, where, gurobi.CB_MIPSOL_SOL, modelData.VarCount)
		if err != nil {
			//log.Println(err)
			Log(1, err.Error())
		}

		objval, err := gurobi.CbGetDbl(cbdata, where, gurobi.CB_MIPSOL_OBJ)
		if err != nil {
			//log.Printf("Error retrieving objval: %s\n", os.Args[1])
			Log(1, "Error retrieving objval: %s", err.Error())
			return 0
		}

		nodeAss := ExtractNodeMatrix(sol, N, M, modelData.XStart)
		for c := 0; c < len(modelData.GCuts); c++ {
			cut := modelData.GCuts[c]
			if cut == CUT_SEC {
				solA := ExtractEdgeMatrix(sol, N, M, modelData.YStart, modelData.GMastermodel)
				var subtours [][]int
				//for each machine, check if there are invalid subtours that do not contain the depot and save those
				//to derive SECs from them later
				for i := 0; i < M; i++ {
					//log.Printf("Looking for subtours in edgeMatrix %d : \n%v\n",i,solA[i])
					Log(4, "Looking for subtours in edgeMatrix %d : \n%v\n", i, solA[i])
					tour, isTourInvalid := Findsubtour(solA[i])
					nodeCount := 0
					for j := 0; j < len(nodeAss[i]); j++ {
						if nodeAss[i][j] == 1 {
							nodeCount++
						}
					}
					if isTourInvalid {
						subtours = append(subtours, tour)
					}
				}
				if len(subtours) > 0 {
					//Add the SECs
					secInd, secVal, op, rhs := getSECs(modelData, subtours, N, M, modelData.YStart)
					for i := 0; i < len(secInd); i++ {
						err = gurobi.CbLazy(cbdata, len(secInd[i]), secInd[i], secVal[i], op, rhs[i])
						if err != nil {
							//log.Println(err)
							Log(1, err.Error())
						}
					}
				}
			}
		}

		heurSolObj := 0
		heurSol := make([][]int, M)
		for i := 0; i < M; i++ {
			indx := make([]int, 0)
			for n := 0; n < len(nodeAss[i]); n++ {
				if nodeAss[i][n] == 1 {
					indx = append(indx, n)
				}
			}
			d := make([][]int, len(indx))
			for j := 0; j < len(d); j++ {
				a := indx[j]
				d[j] = make([]int, len(indx))
				for k := 0; k < len(d); k++ {
					b := indx[k]
					if j == k {
						continue
					}
					d[j][k] = modelData.EdgeWeights[a][b] * modelData.TravelSpeeds[i]
				}
			}

			var (
				tour       []int
				tourLength int
				subtours   [][]int
			)
			if len(d) == 2 {
				//there is only 1 node + the depot assigned to this machine, so we dont need to solve the tsp
				tourLength = d[0][1] + d[1][0]
				tour = []int{0, 1}
			} else {
				tour, tourLength, subtours = tsp.SolveTSP(d, modelData.GEnv)
				if tour == nil || tourLength < 0 {
					Log(1, "The tsp for the subproblem was nil...Why?")
					Log(1, Print2DArray(d))
				}
				/*else {
					log.Printf("TSP-Tour for Machine %d:\n",d)
					print2DArray(d)
				}*/
			}

			if heurSolObj < tourLength {
				heurSolObj = tourLength
			}
			//translate machine tour to global indxs
			for k := 0; k < len(tour); k++ {
				tour[k] = indx[tour[k]]
			}
			//translate tsp sub-tours to global indxs
			for j := 0; j < len(subtours); j++ {
				for k := 0; k < len(subtours[j]); k++ {
					subtours[j][k] = indx[subtours[j][k]]
				}
			}

			heurSol[i] = tour

			Log(3, "\nSolution of the subproblem yielded a tour %v, with length %d!", tour, tourLength)

			if tour != nil && tourLength >= 0 && tourLength > int(objval+0.5) { //since the solution is an integer, we add some float values to avoid numerical errors.
				//log.Printf("Invalid solution found, CMax is %d but must be >= %d. Cutting it off...",int(objval+0.5),tourLength);
				/*ind, val, op, rhs := getBendersCutV1(modelData,i,modelData.EdgeWeights,tour,tourLength)
				// Add the benders cut
				err = gurobi.CbLazy(cbdata, len(ind), ind, val, op, rhs)
				if err != nil {
					log.Println(err)
				}*/
				for c := 0; c < len(modelData.GCuts); c++ {
					cut := modelData.GCuts[c]
					if cut == CUT_SEC {
						/*
							//TODO: we do not add the subtours from the subproblem - instead we found active subtours in the masterproblem already
							//But exclude the ones containing the depot, since we cannot forbid those
							var filteredSubtours [][]int
							var (
								inds [][]int32
								vals [][]float64
								op   int8
								rhs  []float64
							)
							for j := 0; j < len(subtours); j++ {
								hasDepot := false
								for k := 0; k < len(subtours[j]); k++ {
									if subtours[j][k] == 0 {
										hasDepot = true
										break
									}
								}
								if !hasDepot {
									filteredSubtours = append(filteredSubtours, subtours[j])
								} else {
									Log(4,"SEC for subtour %v from the subproblem will not be included in the Masterproblem (filtered out)",subtours[j])
								}
							}
							if len(filteredSubtours) < 1 {
								Log(3, "Supposed to add SECs from the subproblem, but there are none, that do not include the depot!")
							} else {
								inds, vals, op, rhs = getSECs(modelData, filteredSubtours, N, M, modelData.YStart)
							}
							for j := 0; j < len(inds); j++ {
								err = gurobi.CbLazy(cbdata, len(inds[j]), inds[j], vals[j], op, rhs[j])
								if err != nil {
									//log.Println(err)
									Log(1, err.Error())
								}
							}
						*/
					} else {
						//also cut it for all other vehicles with the same travel speed
						for s := 0; s < len(modelData.TravelSpeeds); s++ {
							if modelData.TravelSpeeds[s] == modelData.TravelSpeeds[i] {
								var (
									ind []int32
									val []float64
									op  int8
									rhs float64
								)
								if cut == CUT_BEND_V1 {
									ind, val, op, rhs = getBendersCutV1(modelData, s, tour, tourLength)
								} else if cut == CUT_BEND_V2 {
									ind, val, op, rhs = getBendersCutV2(modelData, s, tour, tourLength)
								} else if cut == CUT_BEND_V3 {
									ind, val, op, rhs = getBendersCutV3(modelData, s, tour, tourLength)
								} else if cut == CUT_BEND_V4 {
									ind, val, op, rhs = getBendersCutV4(modelData, s, tour, tourLength)
								} else if cut == CUT_BEND_V5 {
									ind, val, op, rhs = getBendersCutV5(modelData, s, tour, tourLength)
								} else if cut == CUT_BEND_V6 {
									ind, val, op, rhs = getBendersCutV6(modelData, s, tour, tourLength)
								} else {
									continue
								}
								// Add the benders cut
								err = gurobi.CbLazy(cbdata, len(ind), ind, val, op, rhs)
								if err != nil {
									//log.Println(err)
									Log(1, err.Error())
								}
							}
						}
					}
				}
			} else {
				//the solution does not invalidate the master solution
				Log(4, "The Integer-Solution seems to be valid! No Cuts added!")
				continue
			}
		}
		if modelData.BestSol.Obj > heurSolObj {
			Log(2, "Current best objective was %d, setting it to %d now\n", modelData.BestSol.Obj, heurSolObj)
			modelData.BestSol.Obj = heurSolObj
			modelData.BestSol.Routes = heurSol

			if int(objval+0.5) == heurSolObj {
				//The current master-solution has the same objval as the calculated sequences from TSP, so the value has been used already before we get the chance to set the solution!
				modelData.NewBestSol = false
			} else if int(objval+0.5) > 0 && int(objval+0.5) < heurSolObj {
				//The heuristic solution is worse than the current objval, which means we added some benders cuts
				Log(2, "Found new best solution with value %d, while the master solution was invalid", heurSolObj)

				modelData.NewBestSol = true
			} else {
				//The heuristic solution was better, than the master solution (this can happen??) HOW come??
				Log(2, "Found new best solution with value %d, which is even better than the current master solution!", heurSolObj)
				modelData.NewBestSol = true
			}
		}
	}

	if where == gurobi.CB_MIPNODE {
		if modelData.NewBestSol {
			objbst, err := gurobi.CbGetDbl(cbdata, where, gurobi.CB_MIPNODE_OBJBST)
			if err != nil {
				Log(1, "Couldn't retrieve the obj_best in the callback: %s\n", err.Error())
				return 0
			}
			if int(objbst+0.5) > 0 && int(objbst+0.5) <= modelData.BestSol.Obj {
				Log(2, "Current obj %d is already better than the heuristic solution %d . Skipping...\n", int(objbst+0.5), modelData.BestSol.Obj)
				modelData.NewBestSol = false
				return 0
			}
			Log(2, "Currently setting new heuristic solution with obj-value %d replacing the current bestobj %d \n", modelData.BestSol.Obj, int(objbst+0.5))
			solution := make([]float64, modelData.VarCount)

			//set the objective
			solution[modelData.CMax] = float64(modelData.BestSol.Obj)

			//set X and Y-Variables
			sX := ""
			sY := ""
			for i := 0; i < len(modelData.BestSol.Routes); i++ {
				prev := 0
				for j := 0; j < len(modelData.BestSol.Routes[i]); j++ {
					act := modelData.BestSol.Routes[i][j]
					solution[GetNodeIndex(i, act, N, modelData.XStart)] = 1.0
					sX += fmt.Sprintf("%s = %d, ", modelData.VarNames[GetNodeIndex(i, act, N, modelData.XStart)], 1)
					if prev != 0 || act != 0 {
						solution[GetEdgeIndex(i, prev, act, N, modelData.YStart, modelData.GMastermodel)] = 1.0
						sY += fmt.Sprintf("%s = %d, ", modelData.VarNames[GetEdgeIndex(i, prev, act, N, modelData.YStart, modelData.GMastermodel)], 1)
					}
					prev = act
				}
				v := 1.0
				if modelData.GMastermodel != MASTERMODEL_ATSP && len(modelData.BestSol.Routes[i]) == 2 {
					v = 2.0
				}
				solution[GetEdgeIndex(i, prev, 0, N, modelData.YStart, modelData.GMastermodel)] = v
				sY += fmt.Sprintf("%s = %d, ", modelData.VarNames[GetEdgeIndex(i, prev, 0, N, modelData.YStart, modelData.GMastermodel)], int(v))
			}
			//set the solution
			val, err := gurobi.CbSolution(cbdata, solution)

			//check the error and objv
			if err != nil {
				Log(1, "Couldn't set the heuristic solution: %s\n", err.Error())
			} else {
				modelData.NewBestSol = false
				if int(val) > 0 {
					Log(2, "New best solution with value : %d set!\n", int(val))
				} else {
					Log(1, "Something went wrong when setting the solution!")
					Log(1, "We tried to set routes: \n%s", Print2DArray(modelData.BestSol.Routes))
				}
			}
		}
	}
	return 0
}

func getSECs(model *MTSPModel, subtours [][]int, N int, M int, start int) (secInd [][]int32, secVal [][]float64, op int8, rhs []float64) {
	for _, stour := range subtours {
		var (
			ind []int32
			val []float64
		)

		/* Add a subtour elimination constraint - those are valid for all vehicles*/
		for i := 0; i < M; i++ {
			for j := 0; j < len(stour); j++ {
				st := j + 1
				if model.GMastermodel == MASTERMODEL_ATSP {
					st = 0
				}
				for k := st; k < len(stour); k++ {
					if k == j {
						continue
					}
					ind = append(ind, int32(GetEdgeIndex(i, stour[j], stour[k], N, model.YStart, model.GMastermodel)))
					val = append(val, 1.0)


				}
				//TODO: trying SECs based on selected nodes
				ind = append(ind, int32(GetNodeIndex(i, stour[j], N, model.XStart)))
				val = append(val, -1.0)
			}
		}
		CutsSECCount++
		Log(3, "Adding SEC nr.%d for subtour: %v", CutsSECCount, stour)
		secInd = append(secInd, ind)
		secVal = append(secVal, val)
		//TODO: trying SECs based on selected nodes
		rhs = append(rhs, float64(-1))
		//rhs = append(rhs, float64(len(stour)-1))
	}
	return secInd, secVal, gurobi.LESS_EQUAL, rhs
}

//These must be valid, can't imagine it would cutoff any feasible solutions...
func getBendersCutV1(model *MTSPModel, i int, tour []int, tourLength int) (ind []int32, val []float64, op int8, rhs float64) {
	thetaSum := 0
	for j := 1; j < len(tour); j++ {
		/*
			prev := j-1
			next := (j+1) % len(tour)
			theta := model.EdgeWeights[tour[prev]][tour[j]] + model.EdgeWeights[tour[j]][tour[next]]
		*/
		max := 0
		maxK := 0
		for k := 0; k < len(tour); k++ {
			if j == k {
				continue
			}
			next := model.EdgeWeights[tour[j]][tour[k]] * model.TravelSpeeds[i]
			if next > max {
				max = next
				maxK = k
			}
		}
		Log(4, "Longest edge from %d is to %d with %d", tour[j], tour[maxK], max)
		theta := max * 2 //2x the distance to the furthest node in the same assignment
		thetaSum += theta
		ind = append(ind, int32(GetNodeIndex(i, tour[j], model.N, model.XStart)))
		val = append(val, float64(-theta))
	}
	ind = append(ind, int32(model.CMax))
	val = append(val, 1.0)

	bCut := fmt.Sprintf("Cmax")
	for vn := 0; vn < len(ind)-1; vn++ {
		bCut += fmt.Sprintf(" %d*%s", int(val[vn]), model.VarNames[ind[vn]])
	}
	bCut += fmt.Sprintf(" >= %d - %d", tourLength, thetaSum)
	CutsBendersCount++
	Log(3, "Adding benders cut V1 nr.%d:\n%s\n", CutsBendersCount, bCut)
	return ind, val, gurobi.GREATER_EQUAL, float64(tourLength - thetaSum)
}

//These cuts may not be valid and cut off feasible solutions - use with caution. less restrictive than V3
//since V3 are probably valid, these also should hold
func getBendersCutV2(model *MTSPModel, i int, tour []int, tourLength int) (ind []int32, val []float64, op int8, rhs float64) {
	thetaSum := 0
	for j := 1; j < len(tour); j++ {
		max := 0
		for k := 0; k < len(tour); k++ {
			if j == k {
				continue
			}
			edge := model.EdgeWeights[tour[j]][tour[k]] * model.TravelSpeeds[i]
			if max < edge {
				max = edge
			}
		}
		theta := max + model.EdgeWeights[tour[j]][tour[0]] * model.TravelSpeeds[i]
		thetaSum += theta
		ind = append(ind, int32(GetNodeIndex(i, tour[j], model.N, model.XStart)))
		val = append(val, float64(-theta))
	}
	ind = append(ind, int32(model.CMax))
	val = append(val, 1.0)

	bCut := fmt.Sprintf("Cmax")
	for vn := 0; vn < len(ind)-1; vn++ {
		bCut += fmt.Sprintf(" %d*%s", int(val[vn]), model.VarNames[ind[vn]])
	}
	bCut += fmt.Sprintf(" >= %d - %d", tourLength, thetaSum)
	CutsBendersCount++
	Log(3, "Adding benders cut V4 nr.%d:\n%s\n", CutsBendersCount, bCut)
	return ind, val, gurobi.GREATER_EQUAL, float64(tourLength - thetaSum)
}

//These cuts are less restrictive, than the V4 cuts, so if V4 is valid, those are also valid, but if V4 is not, these might still be invalid
//Those are probably valid though
func getBendersCutV3(model *MTSPModel, i int, tour []int, tourLength int) (ind []int32, val []float64, op int8, rhs float64) {
	thetaSum := 0
	for j := 1; j < len(tour); j++ {
		min := -1
		max := 0
		for k := 0; k < len(tour); k++ {
			if j == k {
				continue
			}
			edge := model.EdgeWeights[tour[j]][tour[k]] * model.TravelSpeeds[i]
			if min < 0 || edge < min {
				min = edge
			}
			if max < edge {
				max = edge
			}
		}
		theta := min + max
		thetaSum += theta
		ind = append(ind, int32(GetNodeIndex(i, tour[j], model.N, model.XStart)))
		val = append(val, float64(-theta))
	}
	ind = append(ind, int32(model.CMax))
	val = append(val, 1.0)

	bCut := fmt.Sprintf("Cmax")
	for vn := 0; vn < len(ind)-1; vn++ {
		bCut += fmt.Sprintf(" %d*%s", int(val[vn]), model.VarNames[ind[vn]])
	}
	bCut += fmt.Sprintf(" >= %d - %d", tourLength, thetaSum)
	CutsBendersCount++
	Log(3, "Adding benders cut V2 nr.%d:\n%s\n", CutsBendersCount, bCut)
	return ind, val, gurobi.GREATER_EQUAL, float64(tourLength - thetaSum)
}

//These cuts seem to be INVALID and cut off feasible solutions - use with caution (adaptation of Tran et al.) with pseudo-process and setup times
func getBendersCutV4(model *MTSPModel, i int, tour []int, tourLength int) (ind []int32, val []float64, op int8, rhs float64) {
	thetaSum := 0
	//we have to calculate the pseudo node duration and pseudo setup-times once again in the context of this node selection
	nodeDur := make([]int, len(tour))
	for j := 0; j < len(tour); j++ {
		min := -1
		for k := 0; k < len(tour); k++ {
			if k == j {
				continue
			}
			edge := model.EdgeWeights[tour[j]][tour[k]] * model.TravelSpeeds[i]
			if min < 0 || edge < min {
				min = edge
			}
		}
		nodeDur[j] = min
	}
	for j := 1; j < len(tour); j++ {
		maxPre := 0
		for k := 0; k < len(tour); k++ {
			if j == k {
				continue
			}
			//edge := model.EdgeWeights[tour[k]][tour[j]]*model.TravelSpeeds[i] - nodeDur[k]
			edge := model.ps[tour[k]][tour[j]] * model.TravelSpeeds[i]
			if maxPre < edge {
				maxPre = edge
			}
		}
		theta := nodeDur[j] + maxPre
		//theta := model.pp[tour[j]] + maxPre
		thetaSum += theta
		ind = append(ind, int32(GetNodeIndex(i, tour[j], model.N, model.XStart)))
		val = append(val, float64(-theta))
	}
	ind = append(ind, int32(model.CMax))
	val = append(val, 1.0)

	bCut := fmt.Sprintf("Cmax")
	for vn := 0; vn < len(ind)-1; vn++ {
		bCut += fmt.Sprintf(" %d*%s", int(val[vn]), model.VarNames[ind[vn]])
	}
	bCut += fmt.Sprintf(" >= %d - %d", tourLength, thetaSum)
	CutsBendersCount++
	Log(3, "Adding benders cut V3 nr.%d:\n%s\n", CutsBendersCount, bCut)
	return ind, val, gurobi.GREATER_EQUAL, float64(tourLength - thetaSum)
}



//These cuts may not be valid and cut off feasible solutions - use with caution
func getBendersCutV5(model *MTSPModel, i int, tour []int, tourLength int) (ind []int32, val []float64, op int8, rhs float64) {
	thetaSum := 0
	for j := 1; j < len(tour); j++ {
		max := 0
		for k := 0; k < len(tour); k++ {
			if j == k {
				continue
			}
			edge := model.EdgeWeights[tour[j]][tour[k]] * model.TravelSpeeds[i]
			if max < edge {
				max = edge
			}
		}
		theta := max
		thetaSum += theta
		ind = append(ind, int32(GetNodeIndex(i, tour[j], model.N, model.XStart)))
		val = append(val, float64(-theta))
	}
	ind = append(ind, int32(model.CMax))
	val = append(val, 1.0)

	bCut := fmt.Sprintf("Cmax")
	for vn := 0; vn < len(ind)-1; vn++ {
		bCut += fmt.Sprintf(" %d*%s", int(val[vn]), model.VarNames[ind[vn]])
	}
	bCut += fmt.Sprintf(" >= %d - %d", tourLength, thetaSum)
	CutsBendersCount++
	Log(3, "Adding benders cut V4 nr.%d:\n%s\n", CutsBendersCount, bCut)
	return ind, val, gurobi.GREATER_EQUAL, float64(tourLength - thetaSum)
}

//These cuts are invalid, because they cut off valid solutions. Can only be used as a heuristic
func getBendersCutV6(model *MTSPModel, i int, tour []int, tourLength int) (ind []int32, val []float64, op int8, rhs float64) {
	thetaSum := 0
	for j := 1; j < len(tour); j++ {
		prev := j - 1
		next := (j + 1) % len(tour)
		theta := model.EdgeWeights[tour[prev]][tour[j]] + model.EdgeWeights[tour[j]][tour[next]] + model.EdgeWeights[tour[prev]][tour[next]]
		thetaSum += theta
		ind = append(ind, int32(GetNodeIndex(i, tour[j], model.N, model.XStart)))
		val = append(val, float64(-theta))
	}
	ind = append(ind, int32(model.CMax))
	val = append(val, 1.0)

	bCut := fmt.Sprintf("Cmax")
	for vn := 0; vn < len(ind)-1; vn++ {
		bCut += fmt.Sprintf(" %d*%s", int(val[vn]), model.VarNames[ind[vn]])
	}
	bCut += fmt.Sprintf(" >= %d - %d", tourLength, thetaSum)
	CutsBendersCount++
	Log(3, "Adding benders cut V4 nr.%d:\n%s\n", CutsBendersCount, bCut)
	return ind, val, gurobi.GREATER_EQUAL, float64(tourLength - thetaSum)
}

func ExtractNodeMatrix(solA []float64, N int, M int, xStart int) [][]int {
	xMat := make([][]int, M)
	for i := 0; i < M; i++ {
		xMat[i] = make([]int, N)
	}
	for i := 0; i < M; i++ {
		for j := 0; j < N; j++ {
			if solA[GetNodeIndex(i, j, N, xStart)] > 0.5 {
				xMat[i][j] = 1
			}
		}
	}
	return xMat
}

func ExtractEdgeMatrix(solA []float64, N int, M int, yStart int, model string) [][][]int {
	yMat := make([][][]int, M)
	for i := 0; i < M; i++ {
		yMat[i] = make([][]int, N)
		for j := 0; j < N; j++ {
			yMat[i][j] = make([]int, N)
		}
		for j := 0; j < N; j++ {
			for k := 0; k < N; k++ {
				if k == j {
					continue
				}
				if solA[GetEdgeIndex(i, j, k, N, yStart, model)] > 0.95 {
					yMat[i][j][k] = 1
				}
			}
		}
	}
	return yMat
}

func CheckSolutionValidity(routes [][]int, d [][]int, s []int, obj int) (bool,string) {
	valid := true
	comment := ""
	for i := 0; i < len(routes); i++ {
		routeLength := 0
		for j := 0; j < len(routes[i]); j++ {
			k := (j + 1) % len(routes[i])
			routeLength += d[routes[i][j]][routes[i][k]] * s[i]
		}
		if routeLength > obj {
			comment = fmt.Sprintf("The computed solution is too long! Is %d but can only be %d!", routeLength, obj)
			valid = false
		}
	}
	return valid,comment
}

func CreateMTSPModel(gurobiEnv *gurobi.Env, d [][]int, s []int, xType int8, yType int8, masterModel string, subtourIneq string) (MTSPModel, error) {
	var err error
	CutsSECCount = 0
	CutsBendersCount = 0
	if gurobiEnv == nil {
		//create the gurobi environment */
		gurobiEnv, err = gurobi.LoadEnv("mtsp_gurobi.log")
		defer gurobiEnv.Free()
		gurobiEnv.SetIntParam("LogToConsole", int32(0))
		defer gurobiEnv.SetIntParam("LogToConsole", int32(1))
	}
	addSubtourIneq := false
	if masterModel == MASTERMODEL_ATSP && subtourIneq == SUBTOURINEQ_MTZ {
		addSubtourIneq = true
	}

	N := len(d)
	M := len(s)
	xCount := M * N //X_ij
	yCount := 0
	if masterModel == MASTERMODEL_ATSP {
		yCount = M * (N * (N - 1))
	} else {
		yCount = M * (((N * N) - N) / 2) //Y_ijk
	}
	cCount := 0
	if addSubtourIneq {
		cCount = N
	}
	varCount := 1 + xCount + yCount + cCount //all variables

	CMax := 0
	xStart := CMax + 1
	yStart := xStart + xCount
	cStart := yStart + yCount

	varType := make([]int8, varCount)

	varType[CMax] = gurobi.INTEGER

	for i := xStart; i < xStart+xCount; i++ {
		varType[i] = xType
	}

	for i := yStart; i < yStart+yCount; i++ {
		varType[i] = yType
		//for symmetric problems with binary variables we reset the edges from depots to be integers further down below
	}

	for i := cStart; i < cStart+cCount; i++ {
		varType[i] = gurobi.INTEGER
	}

	varNames := make([]string, varCount)
	varNames[CMax] = "Cmax"
	counter := xStart
	for i := 0; i < M; i++ {
		for j := 0; j < N; j++ {
			varNames[counter] = fmt.Sprintf("X_%d_%d", i, j)
			counter++
		}
	}
	for i := 0; i < M; i++ {
		for j := 0; j < N; j++ {
			if masterModel == MASTERMODEL_ATSP {
				for k := 0; k < N; k++ {
					if k == j {
						continue
					}
					varNames[counter] = fmt.Sprintf("Y_%d_%d_%d", i, j, k)
					counter++
				}
			} else {
				for k := j + 1; k < N; k++ {
					//Allow the edge variables from the depot to be integers (also have the value 2),
					////so that tours with only 1 node are also possible. Otherwise those will be forbidden
					if j == 0 && yType == gurobi.BINARY{
						edgeIndex := GetEdgeIndex(i,j,k,N,yStart,masterModel)
						varType[edgeIndex] = gurobi.INTEGER
					}
					varNames[counter] = fmt.Sprintf("Y_%d_%d_%d", i, j, k)
					counter++
				}
			}
		}
	}
	if addSubtourIneq {
		for i := 0; i < M; i++ {
			varNames[counter] = fmt.Sprintf("C_%d", i)
			counter++
		}
	}

	objFun := make([]float64, varCount)
	objFun[CMax] = 1.0
	for i := 1; i < len(objFun); i++ {
		objFun[i] = 0.0 //need this because of some random values otherwise
	}
	// Create model
	model, err := gurobiEnv.NewModel("mtsp", int32(varCount), objFun, nil, nil, varType, varNames)
	if err != nil {
		Log(1, err.Error())
		return MTSPModel{}, err
	}
	//defer model.Free()

	// Change objective sense to minimization
	err = model.SetIntAttr(gurobi.INT_ATTR_MODELSENSE, gurobi.MINIMIZE)
	if err != nil {
		Log(1, err.Error())
		return MTSPModel{}, err
	}

	var pp []int
	var ps [][]int
	//TODO: trying out pseudo setup-times and pseudo-process times
	{
		ps = make([][]int, len(d))
		pp = make([]int, len(d))
		//TODO: trying out pseudo setup-times and pseudo-process times
		for j := 0; j < len(d); j++ {
			min := -1
			for k := 0; k < len(d); k++ {
				if k == j {
					continue
				}
				edge := d[j][k]
				if min < 0 || edge < min {
					min = edge
				}
			}
			pp[j] = min
		}
		for j := 0; j < len(d); j++ {
			ps[j] = make([]int, len(d))
			for k := 0; k < len(d); k++ {
				if k == j {
					ps[j][k] = 0
					continue
				}
				ps[j][k] = d[j][k] - pp[j]
			}
		}
	}

	//Add constraints (2) linking to CMax
	{
		Log(2, "Creating and setting constraints <= CMax (2)")
		for i := 0; i < M; i++ {
			ind := make([]int32, 0)
			val := make([]float64, 0)
			for j := 0; j < N; j++ {
				if masterModel == MASTERMODEL_ATSP {
					//TODO: trying out pseudo setup-times and pseudo-process times
					ni := GetNodeIndex(i, j, N, xStart)
					Log(4, "Adding %d*X_{%d %d} at var index %d with name %s", pp[j]*s[i], i, j, ni, varNames[ni])
					ind = append(ind, int32(ni))
					val = append(val, float64(pp[j]*s[i]))
					for k := 0; k < N; k++ {
						if k == j {
							continue
						}
						ei := GetEdgeIndex(i, j, k, N, yStart, masterModel)
						Log(4, "Adding %d*Y_{%d %d %d} at var index %d with name %s", ps[j][k]*s[i], i, j, k, ei, varNames[ei])
						ind = append(ind, int32(ei))
						val = append(val, float64(ps[j][k]*s[i]))
					}
				} else {
					for k := j + 1; k < N; k++ {
						ind = append(ind, int32(GetEdgeIndex(i, j, k, N, yStart, masterModel)))
						val = append(val, float64(d[j][k]*s[i]))
					}
				}
			}
			ind = append(ind, int32(CMax))
			val = append(val, -1.0)

			err = model.AddConstr(ind, val, gurobi.LESS_EQUAL, 0.0, fmt.Sprintf("2_%d", i))
			if err != nil {
				Log(1, "Error adding constraint (2) at i=%d with error: %s\n", i, err.Error())
				return MTSPModel{}, err
			}
		}
	}

	//Add constraints (3) ensuring each node is only visited by exactly one vehicle
	{
		Log(2, "Creating and setting constraints sum_i(Xij) = 1 (3)") //(2)
		for j := 1; j < N; j++ {
			ind := make([]int32, 0)
			val := make([]float64, 0)
			for i := 0; i < M; i++ {
				ind = append(ind, int32(GetNodeIndex(i, j, N, xStart)))
				val = append(val, 1.0)
			}

			err = model.AddConstr(ind, val, gurobi.EQUAL, 1.0, fmt.Sprintf("3_%d", j))
			if err != nil {
				Log(1, "Error adding constraint (3) at j=%d with error: %s\n", j, err.Error())
				return MTSPModel{}, err
			}
		}
	}

	//Add constraints (4) ensuring each vehicle starts at the depot
	{
		Log(2, "Creating and setting constraints Xi0 = 1 (4)") //(4)
		for i := 0; i < M; i++ {
			ind := make([]int32, 1)
			val := make([]float64, 1)

			ind[0] = int32(GetNodeIndex(i, 0, N, xStart))
			val[0] = 1.0

			err = model.AddConstr(ind, val, gurobi.EQUAL, 1.0, fmt.Sprintf("4_%d", i))
			if err != nil {
				Log(1, "Error adding constraint (4) at i=%d with error: %s\n", i, err.Error())
				return MTSPModel{}, err
			}
		}
	}

	if masterModel == MASTERMODEL_ATSP {
		//Add constraints (5.1)  and (5.2) for the asymmetric version ensuring the incoming flow is 1 and the outgoing also 1
		Log(2, "Creating and setting constraints (5.1) Xij = sum_k(Yikj) and (5.2) Xij = sum_k(Yijk)")
		for i := 0; i < M; i++ {
			for j := 0; j < N; j++ {
				ind := make([]int32, 0)
				val := make([]float64, 0)
				ind = append(ind, int32(GetNodeIndex(i, j, N, xStart)))
				val = append(val, -1.0)
				for k := 0; k < N; k++ {
					if k == j {
						continue
					}
					ei := GetEdgeIndex(i, k, j, N, yStart, masterModel)
					Log(4, "Adding Y_{%d %d %d} at var index %d with name %s", i, k, j, ei, varNames[ei])
					ind = append(ind, int32(ei))
					val = append(val, 1.0)
				}
				Log(3, "Adding sum_k(Y_{%d k %d}) = X_{%d %d} with name 5.1_%d_%d", i, j, i, j, i, j)
				err = model.AddConstr(ind, val, gurobi.EQUAL, 0.0, fmt.Sprintf("5.1_%d_%d", i, j))
				if err != nil {
					Log(1, "Error adding constraint (5.1) at i=%d,j=%d: %s\n", i, j, err.Error())
					return MTSPModel{}, err
				}

				ind = make([]int32, 0)
				val = make([]float64, 0)
				ind = append(ind, int32(GetNodeIndex(i, j, N, xStart)))
				val = append(val, -1.0)
				for k := 0; k < N; k++ {
					if k == j {
						continue
					}
					ei := GetEdgeIndex(i, j, k, N, yStart, masterModel)
					Log(4, "Adding Y_{%d %d %d} at var index %d with name %s", i, j, k, ei, varNames[ei])
					ind = append(ind, int32(ei))
					val = append(val, 1.0)
				}
				Log(3, "Adding sum_k(Y_{%d %d k}) = X_{%d %d} with name 5.2_%d_%d", i, j, i, j, i, j)
				err = model.AddConstr(ind, val, gurobi.EQUAL, 0.0, fmt.Sprintf("5.2_%d_%d", i, j))
				if err != nil {
					Log(1, "Error adding constraint (5.2) at i=%d,j=%d: %s\n", i, j, err.Error())
					return MTSPModel{}, err
				}
			}
		}
	} else {
		//Add constraints (5) for the symmetric version
		Log(2, "Creating and setting constraints 2Xij = sum_k(Yikj) + sum_k(Yijk) (5)")
		for i := 0; i < M; i++ {
			for j := 0; j < N; j++ {
				ind := make([]int32, 0)
				val := make([]float64, 0)
				for k := 0; k < N; k++ {
					if k == j {
						continue
					}
					ei := GetEdgeIndex(i, j, k, N, yStart, masterModel)
					Log(3, "Adding Y_{%d %d %d} at var index %d with name %s", i, j, k, ei, varNames[ei])
					ind = append(ind, int32(ei)) //this method flips j and k on its own if needed
					val = append(val, 1.0)
				}
				ind = append(ind, int32(GetNodeIndex(i, j, N, xStart)))
				val = append(val, -2.0)

				err = model.AddConstr(ind, val, gurobi.EQUAL, 0.0, fmt.Sprintf("5_%d_%d", i, j))
				if err != nil {
					Log(1, "Error adding constraint (5) at i=%d,j=%d: %s\n", i, j, err.Error())
					return MTSPModel{}, err
				}
			}
		}
	}


	{
		Log(2, "Adding Global SEC per vehicle based on node assignments")
		for i := 0; i < M; i++ {
			var (
				ind []int32
				val []float64
			)
			for j := 1; j < N; j++ {
				st := j + 1
				for k := st; k < N; k++ {
					ind = append(ind, int32(GetEdgeIndex(i, j, k, N, yStart, masterModel)))
					val = append(val, 1.0)

				}
				//TODO: trying SECs based on selected nodes??
				ind = append(ind, int32(GetNodeIndex(i, j, N, xStart)))
				val = append(val, -1.0)
			}
			//TODO: trying SECs based on selected nodes??
			err = model.AddConstr(ind, val, gurobi.LESS_EQUAL, -1.0, fmt.Sprintf("SEC_global_%d",i))
			if err != nil {
				Log(1, "Error adding global SEC for vehicle %d with error: %s\n", i, err.Error())
				return MTSPModel{}, err
			}
		}
	}

	if addSubtourIneq {
		//Add constraints (6) as MTZ
		Log(2, "Creating and setting MTZ constraints C_k - C_j + V(1-Y_ijk) >= c_jk*s_i (6)")
		//log.Println("Creating and setting MTZ constraints C_k - C_j + V(1-Y_ijk) >= c_jk*s_i (6)") //(6)
		{
			ind := []int32{int32(cStart)}
			val := []float64{1.0}
			err = model.AddConstr(ind, val, gurobi.EQUAL, 0, fmt.Sprintf("6_%d", 0))
			if err != nil {
				Log(1, "Error adding MTZ-constraint for depot: %s", err.Error())
				return MTSPModel{}, err
			}
		}
		V := 65000.0
		count := 1
		for i := 0; i < M; i++ {
			for j := 0; j < N; j++ {
				for k := 1; k < N; k++ {
					if k == j {
						continue
					}
					ind := make([]int32, 3)
					val := make([]float64, 3)
					ind[0] = int32(cStart + k)
					val[0] = 1.0
					ind[1] = int32(cStart + j)
					val[1] = -1.0
					ind[2] = int32(GetEdgeIndex(i, j, k, N, yStart, masterModel))
					val[2] = V * -1.0

					err = model.AddConstr(ind, val, gurobi.GREATER_EQUAL, float64(d[j][k]*s[i])-V, fmt.Sprintf("6_%d", count))
					if err != nil {
						Log(1, "Error adding MTZ constraint at i=%d, j=%d, k=%d: %s\n", i, j, k, err.Error())
						return MTSPModel{}, err
					}
					count++
				}
			}
		}
	} else {
		Log(2, "No subtour inequalitie classes will be added to the master-problem!")
	}

	// Must set LazyConstraints parameter when using lazy constraints

	err = model.SetIntParam(gurobi.INT_PAR_LAZYCONSTRAINTS, 1)
	if err != nil {
		log.Println(err)
		return MTSPModel{}, err
	}

	mtspModel := MTSPModel{GModel: model, GEnv: gurobiEnv, GMastermodel: masterModel, BestSol: MTSPSolution{Obj: math.MaxInt32}, NewBestSol: false, EdgeWeights: d, TravelSpeeds: s, pp: pp, ps: ps, N: N, M: M, VarNames: varNames, CMax: CMax, XStart: xStart, YStart: yStart, XCount: xCount, YCount: yCount, VarCount: varCount}

	return mtspModel, nil
}
