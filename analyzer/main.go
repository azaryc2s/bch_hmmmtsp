package main

import (
	"encoding/json"
	"fmt"
	"git.solver4all.com/azaryc2s/mtsp"
	"io/ioutil"
	"log"
	"math"
	"os"
	"strings"
)

func main() {
	if len(os.Args) < 2 {
		log.Printf("No arguments passed!")
		return
	}
	dirName := os.Args[1]
	dir, err := ioutil.ReadDir(dirName)
	if err != nil {
		log.Printf("Couldn't open directory %s: %s\n", os.Args[1], err.Error())
		return
	}
	fmt.Printf("Name,Optimal,Time,CMax_Obj,LBound,Gap,Dimension,Comment\n")
	for _, f := range dir {
		fileName := dirName + "/" + f.Name()
		if strings.Contains(fileName, ".json") {
			inst := mtsp.MTSPInstance{}
			instStr, err := ioutil.ReadFile(fileName)
			if err != nil {
				log.Printf("Couldn't read %s: %s\n", f.Name(), err.Error())
				return
			}
			err = json.Unmarshal(instStr, &inst)
			if err != nil {
				log.Printf("Couldn't parse %s: %s\n", f.Name(), err.Error())
				return
			}
			var sol mtsp.MTSPSolution
			if inst.Solution == nil {
				fmt.Printf("No solution for %s\n",inst.Name)
				continue
			}
			sol = *inst.Solution
			inst.EdgeWeights = mtsp.CalcEdgeDist(inst.NodeCoordinates,inst.EdgeWeightType)
			solValid, validComment := mtsp.CheckSolutionValidity(sol.Routes,inst.EdgeWeights,inst.TravelSpeeds,sol.Obj)
			if !solValid {
				sol.Comment += fmt.Sprintf("%s %s",sol.Comment,validComment)
			}
			gap := math.Round((float64(sol.Obj-sol.LBound) / float64(sol.LBound)) * 1000) / 1000.0
			fmt.Printf("%s,%t,%s,%d,%d,%.4f,%d,%s\n", inst.Name, sol.Optimal, sol.Time, sol.Obj, sol.LBound, gap, inst.NodeCount, sol.Comment)
		}
	}

}
