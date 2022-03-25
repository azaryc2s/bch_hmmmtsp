package mtsp

import (
	"log"
	"os"
)

var (
	logSpam *log.Logger
	logDebug *log.Logger
	logInfo *log.Logger
	logErr *log.Logger
	maxLvl int
)

func InitLoggers(logLvl int){
	maxLvl = logLvl
	logSpam = log.New(os.Stdout, "SPAM", log.Ldate|log.Ltime|log.Lshortfile)
	logDebug = log.New(os.Stdout, "DEBUG", log.Ldate|log.Ltime|log.Lshortfile)
	logInfo = log.New(os.Stdout, "INFO", log.Ldate|log.Ltime|log.Lshortfile)
	logErr = log.New(os.Stdout, "ERROR", log.Ldate|log.Ltime|log.Lshortfile)
}

func Log(msgLvl int, printF string, args ...interface{}){
	if msgLvl > maxLvl {
		return
	}
	switch msgLvl {
	case 1:
		logErr.Printf(printF,args...)
	case 2:
		logInfo.Printf(printF,args...)
	case 3:
		logDebug.Printf(printF,args...)
	case 4:
		logSpam.Printf(printF,args...)
	}
}