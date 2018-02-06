#!/usr/bin/python
import os
from tweetBot import tweetBot as bot

if __name__ == "__main__":

	iterations = 1
	params = {"simTime" : 400,
			"numberOfLteNodes" : 1,
			"numberOfWifiNodes" : 1,
			"distance" : 60,
			"OFF" : 4,
			"seed" : 1}

	cmd = "./waf --cwd=p2pSmartest/mixed --run \"scratch/p2pMixedScenario "

	for ue in range(10,20):
		for stas in [863]:
			for i in range (0, iterations):
				params["numberOfLteNodes"] = ue
				params["seed"] += i
				params["numberOfWifiNodes"] = stas
				final = ""
				for k,v in params.items():
					final += " --%s=%s" % (k,v)
				shoot = cmd + final + "\""
				print shoot
				os.system(shoot)
	b = bot()
	b.sendStringMessage("@L_SR, p2pSmartest mixed growing UEs from 10 to 20, 863 stations per WiFi AP, OFF=4s")
