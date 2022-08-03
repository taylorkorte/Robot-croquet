# Joe Rottner is a software Legend, and of course this code works. Why wouldn't it? No need to debug anything it's definitely perfect already. 
import cv2 as cv
import numpy as np
import h5py
import tensorflow as tf
from time import sleep

class Digit:
	# example: 
	#
	# digit = Digit()
	# if (process_webcam()):
	# 	most_likely = digit.first
	# 	percent_likely = digit.fist_percent
	# else: 
	# 	retry above after moving <- pseudocode

	def __init__(self):
		self.first = 0 # most likely digit identified
		self.second = 0 # probability of digit being the most likely digit identified
		self.first_percent = 0 # next most likely digit
		self.second_percent = 0 # probability of digit being next most likely digit identified

	# Updates Class Variables by Default on Call. Returns True if Successful. Returns False if Fails.
	def process_webcam(self):

		def most_likely_output(input):
		    index = 0
		    for i in range (1,6):
		        if input[0][i] > input[0][index]: index = i
		    return index + 1

		def next_most_likely_output(input,ignore):
		    index = 0
		    for i in range (1,6):
		        if input[0][i] > input[0][index] and index == (ignore - 1): index = i
		    return index + 1

		# Returns True if specified position is within cleaning_res (Global Variable) of image edge. Otherwise returns false
		def on_image_edge(image, position_arg):
			if ((position_arg[0] - 20) < 0) or ((position_arg[0] + 20) >= image.shape[0]):
				return True
			if ((position_arg[1] - 20) < 0) or ((position_arg[1] + 20) >= image.shape[1]):
				return True
			return False

		# Checks to see if within the pixel box of length 2 * res centered at starting position, position_arg, if there are any white pixels touching the edge
		# Returns true if there are pixels at the boundary, otherwise returns false
		# Used in practice to identify stand-alone splotches (noise) for cleaning, such that it may be removed
		def check_border_for_edges(image,position_arg,res):
			for row in range(position_arg[0] - res,position_arg[0] + res + 1):
				if (row == position_arg[0] - res) | (row == position_arg[0] + res):
					for col in range(position_arg[1] - res, position_arg[1] + res + 1):
						if not on_image_edge(image,position_arg):
							if image[row][col] == 255: 
								return True
				else:
					for col in range(position_arg[1] - res, position_arg[1] + res + 1, 2 * res):
						if not on_image_edge(image,position_arg):
							if image[row][col] == 255:
								return True
			return False


		# Cleans the edge detection image by iterating through all white pixels, checking their boundaries, and setting the pixels to black if they are identified
		# as being stand-alone, isolated noise rather than a continuous part of the digit
		def clean_edges(image,res):
			points = np.argwhere(image==255)
			for row, col in points:
				if not check_border_for_edges(image,(row,col),res) and not on_image_edge(image,(row,col)):
					image[row][col] = 0

		# Returns true if the average of the two rows above the start and end coloumn position in the specified row is greater than a nominal value
		# Returns false otherwise
		def line_above(image,row,start,end):
			if row == 0: return True
			sum = 0
			for col in range(start,end + 1):
				sum += image[row-1][col]
				sum += image[row-2][col]
			return (sum / (2*(end - start + 1))) > 80

		# Returns a filled image from an image outline
		def fill_image(image):
			img_cpy = image.copy() # create a copy of the input image
			points = np.argwhere(img_cpy==255) # create a list of points that have value 255 (white)
			cache = (-1,-1) # used in function to identify the start of a fill train
			last_filled = (-4,-4) # used in function to identify last filled pixel within row
			for row, col in points:
				if (row != last_filled[0]): last_filled = (-4,4) # ensures last_filled is consistent with current row
				if (row == cache[0]) and ((col - cache[1]) > 1): # if the row of two points is the same, and their horizontal distance is greater than one
					start = cache[1] # identify start position
					end = col # identify end position
					if line_above(img_cpy,row,start,end): # if there are lines above the potential fill train, that proves we are filling in the right spot
						for col_iter in range(start,end + 1):
							img_cpy[row][col_iter] = 255 # set pixels in fill train to white
						cache = (row,col)
						last_filled = (row,col)
					elif (col - last_filled[1]) > 6: cache = (row,col) # if the distance to the last filled image in the row is greater than 6, then it probably is safe to fill the next pair of pixels
				elif (col - last_filled[1]) > 6: cache = (row,col)
			return img_cpy

		def image_fill_v2(image):
			blur = cv.GaussianBlur(image, (5,3), 0)
			points = np.argwhere(blur > 0)
			for row, col in points:
			    blur[row][col] = 255
			return blur

		def image_fill_v3(image):
			blur = cv.GaussianBlur(image, (3,3), 0)
			points = np.argwhere(blur > 0)
			for row, col in points:
			    blur[row][col] = 255
			return blur

		# Fills in blank spots within filled image. For every black pixel in the image, this function sets the pixel to white if more than 6 of its nearest neighbors are also white pixels
		def image_clean_fill(image):
			for row in range(1,image.shape[0] - 1):
				for col in range(1,image.shape[1] - 1):
					if (image[row][col] == 255): continue
					score = 0
					score = score + (image[row - 1][col - 1] == 255) + (image[row - 1][col] == 255) + (image[row - 1][col + 1] == 255)
					score = score + (image[row][col - 1] == 255) + (image[row][col] == 255) + (image[row][col + 1] == 255)
					score = score +  (image[row + 1][col - 1] == 255) + (image[row + 1][col] == 255) + (image[row + 1][col + 1] == 255)
					if score > 6: 
						image[row][col] = 255

		def connectTwoPoints(x1, y1, x2, y2, inputGraph):
				# First, find which point is further away
				x, y = x1, y1
				length = abs((x2 - x1) if abs(x2 - x1) > abs(y2 - y1) else (y2 - y1))
				dx = (x2 - x1)/float(length)
				dy = (y2 - y1)/float(length)
				inputGraph[round(x)][round(y)] = 1

				# Once we determine which point is further away, we can begin moving from the closer
				# point to the further point, dx and dy represent the change of x and y
				for i in range(length):
					x += dx
					y += dy
					inputGraph[round(x)][round(y)] = 1
				# inputGraph stores the values of the connected points as well as the points being connected
				return inputGraph

		def row_has_white(image,row):
			for col in range(image.shape[1]):
				if image[row][col] == 255: return True
			return False

		def delete(image,start,end):
			for row in range(start, end+1):
				for col in range(image.shape[1]):
					image[row][col] = 0
			return

		def border_crop(image):
			points = np.argwhere(image == 255)
			for row, col in points:
				if col < round(image.shape[1] / 4) or col > image.shape[1] - round(image.shape[1] / 4): image[row][col] = 0
			switch = False
			start_row = -1
			for row in range(image.shape[0] - 3):
				if row_has_white(image,row):
					if not switch: 
						switch = True
						start_row = row
				elif switch:
					if row_has_white(image,row+1) or row_has_white(image,row+2) or row_has_white(image,row+3): continue
					else:
						switch = False
						dist = row - start_row
						if dist < image.shape[0] / 5:
							delete(image,start_row,row)
				else: continue
			if image.shape[0] - start_row < image.shape[0] / 3: delete(image,start_row,image.shape[0] - 1)

		def hard_crop(image):
		    upper_lim = round(image.shape[0] / 6)
		    lower_lim = round(image.shape[0] - image.shape[0] / 3)
		    left_lim = round(image.shape[1] / 4)
		    right_lim = round(image.shape[1] - image.shape[1] / 4)
		    return image[upper_lim:lower_lim,left_lim:right_lim]

		def get_index_card(image,image_out):
		    counter = 0
		    dist_high = 0
		    location_high = (0,0,0)
		    for row in range(0,image.shape[0]):
		        position_arg = (row,0)
		        for col in range(0,image.shape[1]):
		            if image[row][col] == 0 and col != image.shape[1] - 1: counter += 1
		            elif counter > 5:
		                counter = 0
		                if row != position_arg[0] or (col - position_arg[1]) < (1.1 * dist_high): 
		                    position_arg = (row,col)
		                else: 
		                    dist_high = col - position_arg[1]
		                    location_high = (row,position_arg[1],col)
		            else: 
		                counter = 0
		                position_arg = (row,col)
		    if dist_high < image.shape[1] / 2: return [[[-1]]]
		    checking_col = location_high[1] + round(image.shape[1] / 20)
		    checking_row = location_high[0] + 10
		    while image[checking_row][checking_col] != 255 and checking_row != (image.shape[0] - 1):
		        checking_row += 1
		    return image_out[location_high[0]:checking_row,location_high[1]:location_high[2]]

		def is_valid_output(cropped):
			# First, check to see if autocropping failed
			if cropped[0][0][0] == -1: return False
			# next check to make sure the image isn't a blank box (i.e. < 5 pixels of white)
			points = np.argwhere(cv.Canny(cropped,100,200) == 255)
			if points.size() < 25: return False
			return True


		def autocrop(image):
			edges = cv.Canny(image,250,350)
			edges_crop = hard_crop(edges)
			image_crop = hard_crop(image)
			blur = cv.GaussianBlur(edges_crop, (9,9), 0)
			points = np.argwhere(blur > 0)
			for row, col in points:
			    blur[row][col] = 255
			temp =  get_index_card(blur,image_crop)
			print("Gate 3")
			if is_valid_output(temp): return temp
			else: return [[[-1]]]

		def findGroupings(grid):
			visited = set()
			max_size = 0
			row, col = len(grid), len(grid[0])
			directions=[(-1,0),(0,1),(1,0),(0,-1),(1,1),(-1,-1),(-1,1),(1,-1)]
			a, b, d, ma, mb, md = 0,0,0,[],[],[]

			def dfs(x, y):
				area = 1
				# For each direction possible...
				for dx, dy in directions:
					# Go in that direction
					nx, ny = x + dx, y + dy
					# Check if that movement is within the bounds of our image and that we haven't visited this pixel before
					if 0 <= nx < row and 0 <= ny < col and (nx,ny) not in visited and grid[nx][ny]:
						visited.add((nx,ny))
						area += dfs(nx,ny)        
				# Find the distance between this given pixel and the start
				# Store these values for all outer pixels
				d = ((((nx - 1 - a)**2) + ((ny + 1 - b)**2))**0.5)
				ma.append(nx-1)
				mb.append(ny+1)
				md.append(d)
				#print('a = %s, b = %s, d = %s'%(ma,mb,md))

				return area
			# For each pixel
			for x in range(row):
				for y in range(col):
					# If the pixel exists, ie, not 0, that means its the start of a grouping
					if grid[x][y] and (x,y) not in visited:
						if (x,y) not in start:
							start.append((x,y))
						visited.add((x,y))
						a, b = x, y
						# Find the point furthest away from the start of the grouping
						max_size = max(dfs(x,y), max_size)
						RealMax = 0
						for i in range(len(md)):
							if md[i] > RealMax:
								RealMax = md[i]
								a = ma[i]
								b = mb[i]
						end.append((a,b))
						ma.clear()
						mb.clear()
						md.clear()
			return max_size

		# ============================== Functions End, Main File Begins ==============================
		image = [[[-1]]]
		counter = 1
		trying_to_find_index_card = True
		while image[0][0][0] == -1 and trying_to_find_index_card:
			print("Autocropping ---- Attempt",counter,":")
			webcam = cv.VideoCapture(0)
			print("this")
			if not webcam.isOpened():
				raise Exception("No image found")
			print("that")
			ret, image = webcam.read()
			webcam.release()
			cv.imshow("Image",image)
			cv.waitKey(1)
			print("Gate 2")
			image = autocrop(image)
			print('Autocrop')
			counter += 1
			if counter > 3: trying_to_find_index_card = False
			elif image[0][0][0] == -1: print("Attempt Failed... Retrying...")

		if trying_to_find_index_card: 
			print("Success, Image Acquired!")
			cv.imshow("Image",image)
			cv.waitKey(0)

			edges = cv.Canny(image,300,450)

			print("Cleaning Image...")
			border_crop(edges)

			# Clean image edges for noise and splotches (ran at 2 different resolutions for better performance)
			clean_edges(edges,10)
			clean_edges(edges,1)
			clean_edges(edges,5)

			border_crop(edges)
			print("Done")
			# Locate digit from cleaned image
			points = np.argwhere(edges==255) # find where the white pixels are
			points = np.fliplr(points) # store them in x,y coordinates instead of row,col indices
			x, y, w, h = cv.boundingRect(points) # create a rectangle around those points
			x, y, w, h = x-10, y-10, w+20, h+20 # make the box a little bigger
			if x < 0: x = 0
			if y < 0: y = 0

			# Make Images Square
			if (w>=h):
			    y -= (w-h) / 2
			    y = round(y)
			    h = w
			else:
			    x -= (h-w) / 2
			    x = round(x)
			    w = h

			# Crop Image with square boundaries
			crop = edges[y:y+h, x:x+w]

			print("Starting DFS for Pixel Correction...")
			graphSize, limit = w - 1, 10
			graph = crop.copy()
			graph2 = [x[:] for x in [[0] * graphSize] * graphSize]

			start, end  = [], []

			findGroupings(graph)

			startX, startY = map(list, zip(*start))
			endX, endY = map(list, zip(*end))

			for i in range(len(startX)):
				for j in range(len(endX)):
					if i == j:
						continue
					# If the end of a grouping is close to the start of any grouping besides itself, connect to it within a limit
					if ((((endX[j] - startX[i])**2) + ((endY[j] - startY[i])**2))**0.5) <= limit:
						connectTwoPoints(startX[i], startY[i], endX[j], endY[j], graph2)  
			# Merge input graph with connections graph
			for i in range(graphSize):
				for j in range(graphSize):
					if graph[i][j] == 1 or graph2[i][j] == 1:
						graph[i][j] = 255

			print("Done")
			# Fill and clean the filled image of blank spaces within
			filled = fill_image(crop)
			image_clean_fill(filled)
			filled2 = image_fill_v2(crop)
			image_clean_fill(filled2)
			filled3 = fill_image(image_fill_v3(crop))
			image_clean_fill(filled3)

			# Resize the image to match CNN input requirements
			block = cv.resize(filled, (24, 24))
			resized = cv.resize(filled, (28, 28))

			block2 = cv.resize(filled2, (24, 24))
			resized2 = cv.resize(filled2, (28, 28))

			block3 = cv.resize(filled3, (24, 24))
			resized3 = cv.resize(filled3, (28, 28))

			for row in range(28):
				for col in range(28):
					if row < 2 or row > 25: resized[row][col] = 0; resized2[row][col] = 0; resized3[row][col] = 0
					elif col < 2 or col > 25: resized[row][col] = 0; resized2[row][col] = 0; resized3[row][col] = 0
					else: resized[row][col] = block[row - 2][col -  2]; resized2[row][col] = block2[row - 2][col -  2]; resized3[row][col] = block3[row - 2][col -  2]

			# Load in model
			hf = h5py.File('data.h5', 'r')
			model = tf.keras.models.load_model(hf)
			hf.close()

			resized = resized.astype("float32") / 255
			resized = np.expand_dims(resized, -1)
			resized = [resized.tolist()] # additional formatting

			resized2 = resized2.astype("float32") / 255
			resized2 = np.expand_dims(resized2, -1)
			resized2 = [resized2.tolist()] # additional formatting

			resized3 = resized3.astype("float32") / 255
			resized3 = np.expand_dims(resized3, -1)
			resized3 = [resized3.tolist()] # additional formatting
			print("Predicting Digits...")
			# ----- Classify image -----
			# Get digit probability weights for each image
			prediction_weights = model.predict(resized)
			print("Predicted first image")
			prediction_weights2 = model.predict(resized2)
			print("Predicted second image")
			prediction_weights3 = model.predict(resized3)
			print("Predicted third image")
			# Classify image with a single integer from probability weights
			out = most_likely_output(prediction_weights)
			next = next_most_likely_output(prediction_weights,out)

			out2 = most_likely_output(prediction_weights2)
			next2 = next_most_likely_output(prediction_weights2,out2)

			out3 = most_likely_output(prediction_weights3)
			next3 = next_most_likely_output(prediction_weights3,out3)

			# Classify test cases as correct or incorrect
			print("Model1 Output: Guessing ",out," with ", prediction_weights[0][out - 1] * 100, "percent confidence. Next most likely output is ",next," with ", prediction_weights[0][next - 1] * 100, "percent confidence."); 

			# Classify test cases as correct or incorrect
			print("Model2 Output: Guessing ",out2," with ", prediction_weights2[0][out2 - 1] * 100, "percent confidence. Next most likely output is ",next2," with ", prediction_weights2[0][next2 - 1] * 100, "percent confidence."); 

			# Classify test cases as correct or incorrect
			print("Model2 Output: Guessing ",out3," with ", prediction_weights3[0][out3 - 1] * 100, "percent confidence. Next most likely output is ",next3," with ", prediction_weights3[0][next3 - 1] * 100, "percent confidence."); 
			self.first = out
			self.first_percentage = prediction_weights[0][out - 1] * 100
			self.second = next
			self.second_percent = prediction_weights[0][next - 1] * 100
			return True

		else: 
			print("Failed to find image. Aborting operation.")
			return False
	
