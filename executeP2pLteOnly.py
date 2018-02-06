#!/usr/bin/python
import os
from tweetBot import tweetBot as bot

if __name__ == "__main__":

	iterations = 3 
	params = {"simTime" : 100,
			"numberOfLteNodes" : 75,
			"numberOfUeClusters" : 1,
			"distance" : 60,
			"seed" : 1}

	cmd = "./waf --cwd=p2pSmartest/mixed --run \"scratch/p2pMixedScenario-lteOnly "
	
	for clusters in range(1, 10, 1):
		for i in range (0, iterations):
			params["numberOfUeClusters"] = clusters
			params["seed"] += i
			final = ""
			for k,v in params.items():
				final += " --%s=%s" % (k,v)
			shoot = cmd + final + "\""
			print shoot
			os.system(shoot)
	b = bot()
	b.sendStringMessage("@L_SR, p2pSmartest LTE-only with clusters is finished")


	
