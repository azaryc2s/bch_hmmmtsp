/* Copyright 2021, Arkadiusz Zarychta, arkadiusz.zarychta@h-brs.de */
/* Copyright 2021, Gurobi Optimization, LLC */

package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"git.solver4all.com/azaryc2s/gorobi/gurobi"
	"git.solver4all.com/azaryc2s/mtsp"
	"git.solver4all.com/azaryc2s/tsp"
	"github.com/shirou/gopsutil/cpu"
	"github.com/shirou/gopsutil/host"
	"github.com/shirou/gopsutil/mem"
	"io/ioutil"
	"math"

	//"log"
	//"os"
	"strings"
	"time"
)

var (
	edgeDist [][]int
	sol      mtsp.MTSPSolution
	pInst    mtsp.MTSPInstance

	cuts        mtsp.ArrayStringFlags
	strat       *string
	inputF      *string
	outputF     *string
	yBounds     *string
	lBoundStrat *string
	subtourIneq *string
	masterModel       *string
	logLvl      *int
)

func main() {
	var err error

	flag.Var(&cuts, "cuts", "List of cuts to be used. Possible:  {BEND_V1 , BEND_V2, BEND_V3, SEC}")
	strat = flag.String("strat", "BCH", "Strategy for solving. BCH (default) or LP")
	masterModel = flag.String("model", "TSP", "How the master problem is to be modelled. Possible: {TSP,ATSP}. Default TSP.")
	subtourIneq = flag.String("subtourIneq", "none", "Define integer-subtour-classes to be added to the master-problem. Default none. Possible: {MTZ}")
	inputF = flag.String("input", "input.json", "Path to the input instance")
	lBoundStrat = flag.String("lbstrat", "none", "Strategy for setting a lower bound. Default none, possible: TSP")
	yBounds = flag.String("yBounds", mtsp.Y_BOUNDS_CONT, "Bounds of the Y-Variables. CONT|BIN")
	outputF = flag.String("output", "", "Path to the output file. By default the input file will be overwritten adding the solution")
	logLvl = flag.Int("log", 2, "Level of the logging output. Higher value is more verbose. Range 1-3")

	flag.Parse()

	hostStat, _ := host.Info()
	cpuStat, _ := cpu.Info()
	vmStat, _ := mem.VirtualMemory()
	sol = mtsp.MTSPSolution{Comment: "", System: mtsp.SysInfo{hostStat.Platform, cpuStat[0].ModelName, fmt.Sprintf("%d GB", (vmStat.Total / 1024 / 1024 / 1024))}}

	instStr, err := ioutil.ReadFile(*inputF)

	mtsp.InitLoggers(*logLvl)
	if err != nil {
		mtsp.Log(1, "At %s: %s\n", *inputF, err.Error())
		return
	}

	err = json.Unmarshal(instStr, &pInst)

	if err != nil {
		mtsp.Log(1, "At %s: %s\n", *inputF, err.Error())
		return
	}
	edgeDist = mtsp.CalcEdgeDist(pInst.NodeCoordinates, pInst.EdgeWeightType)
	pInst.Solution = &sol

	// Create environment
	env, err := gurobi.LoadEnv(fmt.Sprintf("hmmVRP.log"))
	if err != nil {
		mtsp.Log(1, "At %s: %s\n", *inputF, err.Error())
		return
	}
	defer env.Free()
	threads, _ := env.GetIntParam(gurobi.INT_PAR_THREADS)
	sol.Comment = fmt.Sprintf("Solver-Settings: SolverDev: Zarychta, Threads=%d, Strat=%s, yBounds=%s, Cuts=%s", threads, *strat, *yBounds, cuts.String())
	var bounds int8
	if *yBounds == mtsp.Y_BOUNDS_CONT {
		bounds = gurobi.CONTINUOUS
	} else {
		bounds = gurobi.BINARY
	}
	model, err := mtsp.CreateMTSPModel(env, edgeDist, pInst.TravelSpeeds, gurobi.BINARY, bounds, *masterModel, *subtourIneq)
	if err != nil {
		mtsp.Log(1, "At %s: %s\n", *inputF, err.Error())
		return
	}
	if *lBoundStrat == mtsp.LBSTRAT_TSP {
		tspTour, tspLength, _ := tsp.SolveTSP(edgeDist, env)
		mtsp.Log(2, "TSP-Length for this graph is: %d", tspLength)
		sol.TSPLength = tspLength
		var vehSpeedSum float64
		ind := []int32{int32(model.CMax)}
		val := []float64{1.0}
		minEdge := -1
		for j := 0; j < len(edgeDist); j++ {
			for k := j + 1; k < len(edgeDist); k++ {
				if minEdge < 0 || minEdge > edgeDist[j][k] {
					minEdge = edgeDist[j][k]
				}
			}
		}

		maxTourEdge := 0
		for j := 0; j < len(tspTour); j++ {
			k := (j + 1) % len(tspTour)
			if maxTourEdge < edgeDist[j][k] {
				maxTourEdge = edgeDist[j][k]
			}
		}

		tspLength -= (len(pInst.TravelSpeeds) - 1) * maxTourEdge //for each vehicle, we do not use one edge of the tsp in the solution - worst case the longest

		minSpeedF := -1
		for i := 0; i < len(pInst.TravelSpeeds); i++ {
			if minSpeedF < 0 || minSpeedF > pInst.TravelSpeeds[i] {
				minSpeedF = pInst.TravelSpeeds[i]
			}
		}

		for i := 0; i < len(pInst.TravelSpeeds); i++ {
			vehSpeedSum += float64(1) / float64(pInst.TravelSpeeds[i])
			tspLength += minEdge //but we also need one edge more for each vehicle, cause it has to close the cycle
		}
		model.GModel.AddConstr(ind, val, gurobi.GREATER_EQUAL, float64(tspLength)/vehSpeedSum, "tspLBound")
		mtsp.Log(2, "Set the TSPLBound: CMax >= %.2f", float64(tspLength)/vehSpeedSum)
	}
	// Write model to '<fileName>.lp'
	lpName := strings.ReplaceAll(*inputF, ".json", ".lp")
	err = model.GModel.Write(lpName)
	if err != nil {
		mtsp.Log(1, "At %s: %s\n", *inputF, err.Error())
		return
	}
	if *strat == mtsp.STRAT_LP {
		solveBySEC(&model)
	} else if *strat == mtsp.STRAT_BCH {
		model.GCuts = cuts
		solveByBCH(&model)
	} else {
		mtsp.Log(1, "Unsupported strategy : %s\n", *strat)
		return
	}

	solValid, validComment := mtsp.CheckSolutionValidity(sol.Routes, edgeDist, pInst.TravelSpeeds, sol.Obj)
	if !solValid {
		mtsp.Log(1, validComment)
	} else {
		mtsp.Log(1,"The computed solution is valid! ")
	}
	mtsp.Log(2, "Found a hmmVRP-Solution with obj-Value of %d\n", sol.Obj)
}

func captureSolution(model *mtsp.MTSPModel) {
	defer writeSolution()
	gmodel := model.GModel
	// Capture solution information
	optimstatus, err := gmodel.GetIntAttr(gurobi.INT_ATTR_STATUS)
	if err != nil {
		sol.Comment += fmt.Sprintf("Couldn't retrieve optimization status: %s. ", err.Error())
		return
	}

	if optimstatus == gurobi.OPTIMAL {
		sol.Optimal = true
	} else if optimstatus == gurobi.INF_OR_UNBD {
		mtsp.Log(1, "Model for %s is infeasible or unbounded\n", *inputF)
	} else if optimstatus == gurobi.TIME_LIMIT {
		sol.Comment += "Time limit reached"
	} else {
		sol.Comment += "For some reason the optimization stopped before the time limit without an optimal solution"
	}

	objval, err := gmodel.GetDblAttr(gurobi.DBL_ATTR_OBJVAL)
	if err != nil {
		sol.Comment += fmt.Sprintf("Couldn't retrieve the obj-value: %s. ", err.Error())
		return
	}
	if (objval < 0 || objval + 0.5 < objval) && model.BestSol.Obj == 0{
		sol.Obj = math.MaxInt32
	} else if objval > 0 {
		sol.Obj = int(math.Min(objval, float64(model.BestSol.Obj)) + 0.5)
	} else {
		sol.Obj = model.BestSol.Obj
	}
	sol.UBound = sol.Obj

	lb := 0.0
	lb, err = gmodel.GetDblAttr(gurobi.DBL_ATTR_OBJBOUND)
	if err != nil {
		sol.Comment += fmt.Sprintf("Couldn't retrieve the lower-bound-value: %s. ", err.Error())
		mtsp.Log(1, err.Error())
	}
	sol.LBound = int(lb + 0.5)

	// Extract solution
	if model.BestSol.Routes != nil {
		sol.Routes = model.BestSol.Routes
		for i := 0; i < len(sol.Routes); i++ {
			tour := sol.Routes[i]
			length := 0
			for j := 0; j < len(tour); j++ {
				k := (j + 1) % len(tour)
				length += model.EdgeWeights[tour[j]][tour[k]] * model.TravelSpeeds[i]
			}
			sol.RouteCosts = append(sol.RouteCosts, length)
		}
	} else {
		solcount, err := gmodel.GetIntAttr(gurobi.INT_ATTR_SOLCOUNT)
		if err != nil {
			mtsp.Log(1, err.Error())
			return
		}
		if solcount > 0 {
			solA, err := gmodel.GetDblAttrArray(gurobi.DBL_ATTR_X, 0, int32(model.VarCount))
			if err != nil {
				mtsp.Log(1, err.Error())
				return
			}

			yMat := mtsp.ExtractEdgeMatrix(solA, model.N, model.M, model.YStart, model.GMastermodel)
			for i := 0; i < model.M; i++ {
				tour, isTourInvalid := mtsp.Findsubtour(yMat[i])
				if isTourInvalid {
					mtsp.Log(1, "Tour %d is invalid (contains no depot)!\n")
				}
				sol.Routes = append(sol.Routes, tour)
				length := 0
				for j := 0; j < len(tour); j++ {
					k := (j + 1) % len(tour)
					length += edgeDist[tour[j]][tour[k]] * model.TravelSpeeds[i]
				}
				sol.RouteCosts = append(sol.RouteCosts, length)
			}
		}
	}
	mtsp.Log(2, "Added %d SECs and %d Benders-Cuts", mtsp.CutsSECCount, mtsp.CutsBendersCount)
	mtsp.Log(2, "Found Tours with CMax %d : %v \n", sol.Obj, sol.Routes)
}

func solveBySEC(model *mtsp.MTSPModel) {
	gmodel := model.GModel

	/* Must set LazyConstraints parameter when using lazy constraints */
	err := gmodel.SetIntParam(gurobi.INT_PAR_LAZYCONSTRAINTS, 1)
	if err != nil {
		mtsp.Log(1, err.Error())
		return
	}

	err = gmodel.SetCallbackFuncGo(mtsp.LPCallbackMTSP, model)
	if err != nil {
		mtsp.Log(1, err.Error())
		return
	}
	startTime := time.Now()
	// Optimize model
	err = gmodel.Optimize()
	if err != nil {
		mtsp.Log(1, err.Error())
		return
	}

	sol.Time = time.Since(startTime).String()
	mtsp.Log(2, "\n---OPTIMIZATION DONE---\n")
	captureSolution(model)
}

func solveByBCH(model *mtsp.MTSPModel) {
	gmodel := model.GModel

	/* Must set LazyConstraints parameter when using lazy constraints */
	err := gmodel.SetIntParam(gurobi.INT_PAR_LAZYCONSTRAINTS, 1)
	if err != nil {
		mtsp.Log(1, err.Error())
		return
	}

	err = gmodel.SetCallbackFuncGo(mtsp.BCHCallbackMTSP, model)
	if err != nil {
		mtsp.Log(1, err.Error())
		return
	}
	startTime := time.Now()
	// Optimize model
	err = gmodel.Optimize()
	if err != nil {
		mtsp.Log(1, err.Error())
		return
	}

	sol.Time = time.Since(startTime).String()
	mtsp.Log(2, "\n---OPTIMIZATION DONE---\n")
	captureSolution(model)
}

func writeSolution() {
	jsonInst, err := json.MarshalIndent(pInst, "", "\t")
	if err != nil {
		mtsp.Log(1, "At %s: %s\n", *inputF, err.Error())
		return
	}
	jsonInst = []byte(mtsp.SanitizeJsonArrayLineBreaks(string(jsonInst)))
	var fileName string
	if *outputF == "" {
		fileName = *inputF //overwrite the input file
	} else {
		fileName = *outputF //overwrite the input file
	}
	err = ioutil.WriteFile(fileName, jsonInst, 0644)
	if err != nil {
		mtsp.Log(1, "At %s: %s\n", *inputF, err.Error())
		return
	}
}
