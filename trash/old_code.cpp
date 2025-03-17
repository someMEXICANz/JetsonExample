



// void Model::inferTensorRT(float* inputBuffer, float* outputBuffer1, float* outputBuffer2) {
//     int inputIndex = engine->getBindingIndex("000_net");
//     int outputIndex1 = engine->getBindingIndex("016_convolutional");
//     int outputIndex2 = engine->getBindingIndex("023_convolutional");

//     // Set dynamic input dimensions
//     if (!context->setBindingDimensions(inputIndex, nvinfer1::Dims4{32, 3, 320, 320})) {
//         std::cerr << "Failed to set dynamic input dimensions!" << std::endl;
//         return;
//     }

//     // Validate buffers
//     if (buffers[inputIndex] == nullptr || buffers[outputIndex1] == nullptr || buffers[outputIndex2] == nullptr) {
//         std::cerr << "Buffers are uninitialized!" << std::endl;
//         return;
//     }

//     // Copy input to GPU
//     if (cudaMemcpy(buffers[inputIndex], inputBuffer, bufferSizes[inputIndex], cudaMemcpyHostToDevice) != cudaSuccess) {
//         std::cerr << "Failed to copy input data to GPU!" << std::endl;
//         return;
//     }

//     // Execute inference
//     if (!context->enqueueV2(buffers.data(), 0, nullptr)) {
//         std::cerr << "Failed to enqueue inference!" << std::endl;
//         return;
//     }
//     cudaDeviceSynchronize();
//     std::cout << "Inference executed successfully." << std::endl;

//     // Copy outputs to host
//     if (cudaMemcpy(outputBuffer1, buffers[outputIndex1], bufferSizes[outputIndex1], cudaMemcpyDeviceToHost) != cudaSuccess) {
//         std::cerr << "Failed to copy output buffer 1 to host!" << std::endl;
//         return;
//     }
//     if (cudaMemcpy(outputBuffer2, buffers[outputIndex2], bufferSizes[outputIndex2], cudaMemcpyDeviceToHost) != cudaSuccess) {
//         std::cerr << "Failed to copy output buffer 2 to host!" << std::endl;
//         return;
//     }
//     std::cout << "Outputs copied successfully." << std::endl;
// }




// void Model::inferTensorRT(float* inputBuffer, float* outputBuffer1, float* outputBuffer2) {
//     int inputIndex = engine->getBindingIndex("000_net");
//     int outputIndex1 = engine->getBindingIndex("016_convolutional");
//     int outputIndex2 = engine->getBindingIndex("023_convolutional");

//     // Set dynamic input dimensions
//     if (!context->setBindingDimensions(inputIndex, nvinfer1::Dims4{32, 3, 320, 320})) {
//         std::cerr << "Failed to set dynamic input dimensions." << std::endl;
//         return;
//     }

//     // Copy input to GPU
//     cudaMemcpy(buffers[inputIndex], inputBuffer, bufferSizes[inputIndex], cudaMemcpyHostToDevice);

//     // Execute inference
//     if (!context->enqueueV2(buffers.data(), 0, nullptr)) {
//         std::cerr << "Failed to enqueue TensorRT inference." << std::endl;
//         return;
//     }
//     cudaDeviceSynchronize();

//     // Copy outputs to host
//     cudaMemcpy(outputBuffer1, buffers[outputIndex1], bufferSizes[outputIndex1], cudaMemcpyDeviceToHost);
//     cudaMemcpy(outputBuffer2, buffers[outputIndex2], bufferSizes[outputIndex2], cudaMemcpyDeviceToHost);
// }









// // Decode outputs for object detection
// void Model::decodeOutputs(const std::vector<float>& output1, const std::vector<float>& output2,
//                           std::vector<std::tuple<std::string, float, cv::Rect>>& results) {
//     // Assuming output1 and output2 contain detection information:
//     // output1: bounding boxes and classes
//     // output2: confidences
//     // Adjust this logic based on the actual output structure of your model.

//     int numDetections = output1.size() / 6;  // Assuming 6 values per detection
//     for (int i = 0; i < numDetections; ++i) {
//         float x = output1[i * 6 + 0];
//         float y = output1[i * 6 + 1];
//         float w = output1[i * 6 + 2];
//         float h = output1[i * 6 + 3];
//         int classId = static_cast<int>(output1[i * 6 + 4]);
//         float confidence = output1[i * 6 + 5];

//         if (confidence > 0.5) {  // Filter detections by confidence
//             cv::Rect bbox(x, y, w, h);
//             results.emplace_back(labels[classId], confidence, bbox);
//         }
//     }
// }


// void Model::allocateBuffers() {
//     buffers.resize(engine->getNbBindings());
//     bufferSizes.resize(engine->getNbBindings());

//     for (int i = 0; i < engine->getNbBindings(); ++i) {
//         nvinfer1::Dims dims = engine->getBindingDimensions(i);

//         // Handle dynamic dimensions
//         if (dims.nbDims > 0) {
//             std::cout << "Binding " << i << ": " << dims.nbDims << " dimensions: ";
//             size_t size = 1;
//             for (int j = 0; j < dims.nbDims; ++j) {
//                 size *= dims.d[j] > 0 ? dims.d[j] : 1;  // Default to 1 for dynamic dims
//                 std::cout << dims.d[j] << " ";
//             }
//             std::cout << std::endl;

//             bufferSizes[i] = size * sizeof(float);
//             cudaMalloc(&buffers[i], bufferSizes[i]);
//             std::cout << "Buffer " << i << " size: " << bufferSizes[i] << " bytes" << std::endl;
//         } else {
//             std::cerr << "Binding " << i << " has invalid dimensions!" << std::endl;
//         }
//     }
// }

// void Model::allocateBuffers() {

    
//     buffers.resize(engine->getNbBindings());
//     bufferSizes.resize(engine->getNbBindings());

//     for (int i = 0; i < engine->getNbBindings(); ++i) {
//         nvinfer1::Dims dims = engine->getBindingDimensions(i);

//         // Handle dynamic dimensions: Set batch size to 1 if undefined
//         if (dims.d[0] == -1) {
//             dims.d[0] = 1;  // Set batch size to 1
//         }

//         size_t size = 1;
//         for (int j = 0; j < dims.nbDims; ++j) {
//             size *= dims.d[j];
//         }
//         bufferSizes[i] = size * sizeof(float);
//         cudaMalloc(&buffers[i], bufferSizes[i]);

//         std::cout << "Buffer " << i << " size: " << bufferSizes[i] << " bytes" << std::endl;
//     }
// }


// void Model::inferTensorRT(float* inputBuffer, float* outputBuffer1, float* outputBuffer2) {
//     int inputIndex = engine->getBindingIndex("000_net");
//     int outputIndex1 = engine->getBindingIndex("016_convolutional");
//     int outputIndex2 = engine->getBindingIndex("023_convolutional");

//     // Set batch size to 1
//     if (!context->setBindingDimensions(inputIndex, nvinfer1::Dims4{1, 3, 320, 320})) {
//         std::cerr << "Failed to set dynamic input dimensions!" << std::endl;
//         return;
//     }

//     // Validate buffers
//     if (buffers[inputIndex] == nullptr || buffers[outputIndex1] == nullptr || buffers[outputIndex2] == nullptr) {
//         std::cerr << "Buffers are uninitialized!" << std::endl;
//         return;
//     }

//     // Create CUDA stream for asynchronous execution
//     cudaStream_t stream;
//     cudaStreamCreate(&stream);

//     // Copy input to GPU
//     if (cudaMemcpyAsync(buffers[inputIndex], inputBuffer, bufferSizes[inputIndex], cudaMemcpyHostToDevice, stream) != cudaSuccess) {
//         std::cerr << "Failed to copy input data to GPU!" << std::endl;
//         return;
//     }

//     // Execute inference asynchronously
//     if (!context->enqueueV2(buffers.data(), stream, nullptr)) {
//         std::cerr << "Failed to enqueue inference!" << std::endl;
//         return;
//     }

//     // Copy outputs back to host asynchronously
//     if (cudaMemcpyAsync(outputBuffer1, buffers[outputIndex1], bufferSizes[outputIndex1], cudaMemcpyDeviceToHost, stream) != cudaSuccess) {
//         std::cerr << "Failed to copy output buffer 1 from GPU!" << std::endl;
//         return;
//     }
//     if (cudaMemcpyAsync(outputBuffer2, buffers[outputIndex2], bufferSizes[outputIndex2], cudaMemcpyDeviceToHost, stream) != cudaSuccess) {
//         std::cerr << "Failed to copy output buffer 2 from GPU!" << std::endl;
//         return;
//     }

//     // Synchronize the stream
//     cudaStreamSynchronize(stream);

//     // Destroy the CUDA stream
//     cudaStreamDestroy(stream);

//     std::cout << "Inference executed successfully." << std::endl;
// }
// void Model::reshapeOutputs()
// {

//     cv::Mat reshaped1(1,24*10*10,CV_32F,inferOutput1.data());
//     cv::Mat reshaped2(1,24*20*20,CV_32F,inferOutput2.data());

//     reshaped1 = reshaped1.reshape(24,10);
//     reshaped2 = reshaped2.reshape(24,20);

//     ProcessedOutput1 = reshaped1;
//     ProcessedOutput2 = reshaped2;
//     cout << "Reshaped Output 1 : " << reshaped1.rows << " x " << reshaped1.cols 
//               << " x " << reshaped1.channels() << std::endl;
//     cout << "Reshaped Output 2 : " << reshaped2.rows << " x " << reshaped2.cols 
//               << " x " << reshaped2.channels() << std::endl;


//     std::cout << "First 20 values of raw output 1 buffer:\n";
//     for (int i = 0; i < 20; i++) 
//     {
//     std::cout << inferOutput1[i] << " ";
//     }
//     std::cout << "\n";

//     std::cout << "Verifying first few values of reshaped output1:\n";
//     for (int i = 0; i < std::min(10, 5); i++) 
//     {  
//         for (int j = 0; j < std::min(10, 5); j++) 
//         {  // First 5 cols
//             std::cout << "(" << i << "," << j << "): ";
//             for (int c = 0; c < 24; c++) 
//             {
//                 std::cout << reshaped1.at<cv::Vec<float, 3>>(i, j)[c] << " ";
//             }
//             std::cout << "\n";
//         }
//     }

//     std::cout << "First 20 values of raw output 2 buffer:\n";
//     for (int i = 0; i < 20; i++) 
//     {
//         std::cout << inferOutput2[i] << " ";
//     }
//     std::cout << "\n";

//     std::cout << "Verifying first few values of reshaped output2:\n";
//     for (int i = 0; i < std::min(20, 5); i++) 
//     {  
//         for (int j = 0; j < std::min(20, 5); j++) 
//         {  // First 5 cols
//             std::cout << "(" << i << "," << j << "): ";
//             for (int c = 0; c < 24; c++) 
//             {
//                 std::cout << reshaped2.at<cv::Vec<float, 3>>(i, j)[c] << " ";
//             }
//             std::cout << "\n";
//         }
//     }

    

// }


// void Model::reshapeOutputs()
// {
//     cv::Mat rawOutput1(1,1*24*10*10,CV_32F,inferOutput1.data());
//     cv::Mat rawOutput2(1,1*24*20*20,CV_32F,inferOutput2.data());

//     cv::Mat nhwc1;
//     cv::Mat nhwc2;

//     std::vector<cv::Mat> channel1(24);
//     std::vector<cv::Mat> channel2(24);

//     cv::split(rawOutput1, channel1);
//     cv::split(rawOutput2, channel2);

//     cv::vconcat(channel1, nhwc1);
//     cv::vconcat(channel2, nhwc2);

//     // Reshape to NHWC format
//     nhwc1 = nhwc1.reshape(1, {10, 10, 3, 8});
//     nhwc2 = nhwc2.reshape(1, {20, 20, 3, 8});
// }


// cv::Mat Model::reshapeOutputs(const cv::Mat& outputTensor) {
//     // Step 1: Reshape the input Mat to remove batch dim (from 1x24x10x10 to 24x10x10)
//     cv::Mat reshaped = outputTensor.reshape(1, {24, 10, 10});

//     // Step 2: Split the 24 channels into 3 groups of 8 channels each
//     std::vector<cv::Mat> channelGroups(3);
//     for (int i = 0; i < 3; i++) {
//         std::vector<cv::Mat> splitChannels(8);
//         for (int j = 0; j < 8; j++) {
//             splitChannels[j] = reshaped.row(i * 8 + j); // Extract 8 channels per category
//         }
//         cv::vconcat(splitChannels, channelGroups[i]); // Stack them vertically
//     }

//     // Step 3: Merge the 3 groups into the final NHWC format
//     cv::Mat finalOutput;
//     cv::merge(channelGroups, finalOutput); // Merge into (10x10x3x8)

//     return finalOutput; // Now in (10,10,3,8)
// }


// void Model::reshapeOutputs()
// {
//     cv::Mat rawOutput1(1,1*24*10*10,CV_32F,inferOutput1.data());
//     cv::Mat rawOutput2(1,1*24*20*20,CV_32F,inferOutput2.data());
//     cout << "Raw Output 1 dimensions: " << rawOutput1.dims << endl; 
    
//     std::cout << "(";
//     for (int i = 0; i < rawOutput1.dims; i++) 
//     {
//         std::cout << rawOutput1.size[i];
//         if (i < rawOutput1.dims - 1) 
//             std::cout << ", ";
//     }
//     std::cout << ")" << endl;

//     cout << "Raw Output 1 dimensions: " << rawOutput1.dims << endl; 
    
//     std::cout << "(";
//     for (int i = 0; i < rawOutput2.dims; i++) 
//     {
//         std::cout << rawOutput2.size[i];
//         if (i < rawOutput2.dims - 1) 
//             std::cout << ", ";
//     }
//     std::cout << ")" << endl;

    
    

    
// }





// cv::Mat Model::reshapeOutputs(std::vector<float>& output, int H_W) {
//     // Expected input shape: (1, 24, 10, 10) -> NCHW
//     // Desired output shape: (10, 10, 3, 8) -> NHWC

//     int numChannels = 24;
//     int featureChannels = 8;
//     int objectsPerGrid = 3; // 3 objects per grid cell

//     // Convert to a vector of Mat channels
//     std::vector<cv::Mat> channels(numChannels);
//     for (int i = 0; i < numChannels; i++) {
//         channels[i] = cv::Mat(H_W, H_W, CV_32F, (void*)&output[i * H_W * H_W]);
//     }

//     // Split into 3 groups of 8 channels
//     std::vector<cv::Mat> objectChannels(objectsPerGrid);
//     for (int i = 0; i < objectsPerGrid; i++) 
//     {
//         std::vector<cv::Mat> group(channels.begin() + i * featureChannels, channels.begin() + (i + 1) * featureChannels);
//         cv::Mat merged;
//         cv::merge(group, merged);  // Merge 8 channels per object into one Mat
//         objectChannels[i] = merged;
//     }

//     // Stack along depth dimension
//     cv::Mat reshapedOutput;
//     cv::vconcat(objectChannels, reshapedOutput); // Stack 3 objects along height

//     cout << "Reshaped Output dimensions: " << reshapedOutput.dims << endl; 
    
//     std::cout << "(";
//     for (int i = 0; i < reshapedOutput.dims; i++) 
//     {
//         std::cout << reshapedOutput.size[i];
//         if (i < reshapedOutput.dims - 1) 
//             std::cout << ", ";
//     }
//     std::cout << ")" << endl;

//     return reshapedOutput; // Shape should now be (10, 10, 3, 8)
// }




// cv::Mat Camera::preprocessFrames() 
// {
//     // Resize to 320x320
//     cv::Mat resized;
//     cv::resize(color_mat, resized, cv::Size(320, 320));
//     // Normalize to [0, 1]
//     resized.convertTo(resized, CV_32F, 1.0 / 255.0);
//     cv::Mat nchw;
//     std::vector<cv::Mat> channels(3);
//     // Split the channels
//     cv::split(resized, channels);
//     // Concatenate channels in NCHW order
//     cv::vconcat(channels, nchw);

//     // Add batch dimension
//     nchw = nchw.reshape(1, {1, 3, 320, 320});

//     return nchw;
// }







// cv::Mat Model::reshapeOutputs(const std::vector<float>& output, int height, int width) {
//     // Convert to cv::Mat: First reshape to (10, 10, 24)
//     cv::Mat reshapedMat(height, width, CV_32FC(24), (void*)output.data());

//     // Split into 3 channels (anchor boxes)
//     std::vector<cv::Mat> anchorChannels(3);
//     for (int i = 0; i < 3; i++) {
//         anchorChannels[i] = reshapedMat.colRange(i * 8, (i + 1) * 8);
//     }

//     // Merge them into final shape: (height, width, 3, 8)
//     cv::Mat formattedOutput;
//     cv::merge(anchorChannels, formattedOutput);

//     return formattedOutput;
// }



// cv::Mat Camera::preprocessFrames() 
// {
//     // Resize to 320x320
//     cv::Mat resized;
//     cv::resize(color_mat, resized, cv::Size(320, 320));

//     // Normalize to [0, 1]
//     resized.convertTo(resized, CV_32F, 1.0 / 255.0);

//     // Split the channels
//     std::vector<cv::Mat> channels(3);
//     cv::split(resized, channels);

//     // Merge channels into a single Mat (Interleaved format)
//     cv::Mat chw;
//     cv::merge(channels, chw);

//     // Reshape to (1, 3, 320, 320) (NCHW format)
//     chw = chw.reshape(1, {1, 3, 320, 320});

//     return chw;
// }



// cv::Mat Camera::preprocessFrames() 
// {
//     // Resize to 320x320
//     cv::Mat resized;
//     cv::resize(color_mat, resized, cv::Size(320, 320));
//     // Normalize to [0, 1]
//     resized.convertTo(resized, CV_32F, 1.0 / 255.0);
//     cv::Mat nchw;
//     std::vector<cv::Mat> channels(3);
//     // Split the channels
//     cv::split(resized, channels);
//     // Concatenate channels in NCHW order
//     cv::vconcat(channels, nchw);

//     // Add batch dimension
//     nchw = nchw.reshape(1, {1, 3, 320, 320});

//     return nchw;
// }







// std::vector<BoundingBox> ObjectDetection::decodeOutputs(
//     const std::vector<float>& output, int grid_w, int grid_h) {
    
//     std::vector<BoundingBox> detectedBoxes;
//     float original_width = 640;
//     float original_height = 480;

//     int numCells = grid_w * grid_h;
//     int numAnchors = 3;
//     int numClasses = 3;
//     int numElements = 5 + numClasses; 

//     int mask_index = (grid_w == 10) ? 0 : 1; 

//     // Extract correct anchors
//     std::pair<float, float> selected_anchors[3];
//     for (int i = 0; i < 3; i++) {  
//         selected_anchors[i] = anchors[yolo_masks[mask_index][i]];
//     }

//     // Process detections in a single loop
//     for (int anchor = 0; anchor < numAnchors; anchor++) {
//         float anchor_w = selected_anchors[anchor].first;
//         float anchor_h = selected_anchors[anchor].second;

//         for (int i = 0; i < numCells; i++) {
//             int index = (anchor * numCells + i) * numElements;

//             // Decode Center X, Y
//             float centerX = 1 / (1 + exp(-output[index + 0])) + (i % grid_w);
//             float centerY = 1 / (1 + exp(-output[index + 1])) + (i / grid_w);
//             centerX /= grid_w;
//             centerY /= grid_h;

//             // Decode Width & Height
//             float bbox_width = exp(output[index + 2]) * anchor_w;
//             float bbox_height = exp(output[index + 3]) * anchor_h;

//             // Decode Object Confidence
//             float confidence = 1 / (1 + exp(-output[index + 4]));
//             if (confidence < 0.3) continue;  // Early discard if below threshold

//             // Find Best Class
//             float bestClassScore = 0;
//             int bestClassIndex = -1;
//             for (int c = 5; c < 8; c++) {  
//                 float classProb = 1 / (1 + exp(-output[index + c]));  // Apply sigmoid
//                 float finalScore = confidence * classProb;
                
//                 if (finalScore > bestClassScore) {
//                     bestClassScore = finalScore;
//                     bestClassIndex = c - 5;  // Class index (0 = MobileGoal, 1 = RedRing, etc.)
//                 }
//             }

//             // Apply Class-Specific Confidence Threshold
//             if (bestClassScore < obj_thresholds[bestClassIndex]) continue;

//             // Convert to absolute pixel values
//             BoundingBox bbox;
//             bbox.x = centerX * original_width - (bbox_width / 2);
//             bbox.y = centerY * original_height - (bbox_height / 2);
//             bbox.width = bbox_width;
//             bbox.height = bbox_height;
//             bbox.confidence = bestClassScore;
//             bbox.classId = bestClassIndex;

//             detectedBoxes.push_back(bbox);
//         }
//     }

//     return detectedBoxes;
// }



// std::vector<BoundingBox> ObjectDetection::decodeOutputs(const std::vector<float>& output, int grid_w, int grid_h) 
// {

//     // std::cout << "Starting decodeOutputs" << std::endl;
//     std::vector<BoundingBox> detectedBoxes;

//     float original_width = 640;
//     float original_height = 480;

//     int numCells = grid_w * grid_h;
//     int numAnchors = 3; 
//     int numClasses = 3; 
//     int numElements = 5 + numClasses;

//     int mask_index = (grid_w == 10) ? 0 : 1;

//     std::vector<std::pair<float, float>> selected_anchors;
//     for(int i = 0; i < 3; i++)
//     {
//         selected_anchors.push_back(anchors[yolo_masks[mask_index][i]]);
//     }
//     float anchors_tensor[6];
//     for(int i = 0; i < 3; i++)
//     {
//         anchors_tensor[i*2] = selected_anchors[i].first;
//         anchors_tensor[i*2 +1] = selected_anchors[i].second;
//     }


//     for (int anchor = 0; anchor < numAnchors; anchor++)
//     {
//         for(int i = 0; i < numCells; i++)
//         {
//             int index = (anchor * numCells + i ) * numElements;

//             float centerX = 1 / (1 + exp(-output[index + 0]));  
//             float centerY = 1 / (1 + exp(-output[index + 1]));

//             int row = i / grid_w;
//             int col = i % grid_w;
//             centerX = (centerX + col) / grid_w;
//             centerY = (centerY + row) / grid_h;

//             float anchor_w = anchors_tensor[anchor * 2];
//             float anchor_h = anchors_tensor[anchor * 2 + 1];

//             float bbox_width = exp(output[index + 2]) * anchor_w;
//             float bbox_height = exp (output[index +3]) * anchor_h;

//             float confidence = 1 / (1+exp(-output[index+4]));
//             if(confidence < 0.3) continue;

            

//             float bestClassScore = 0;
//             int bestClassIndex = -1; 
//             for(int c = 5; c < 5 + numClasses; c++)
//             {
//                 float classProb = 1 / (1 + exp(-output[index + c]));
//                 if(classProb > bestClassScore) 
//                 {
//                     bestClassScore = classProb;
//                     bestClassIndex = c -5;
//                 }
//             }

//             float finalConfidence = confidence * bestClassScore;
//             if(finalConfidence < 0.3) continue;

//             centerX *= original_width;
//             centerY *= original_height;
//             bbox_width *= original_width;
//             bbox_height *= original_height;

//             BoundingBox bbox;
//             bbox.x = centerX - (bbox_width / 2);
//             bbox.y = centerY - (bbox_height / 2);
//             bbox.width = bbox_width;
//             bbox.height = bbox_height;
//             bbox.confidence = finalConfidence;
//             bbox.classId = bestClassIndex;

//         detectedBoxes.push_back(bbox);

//         }
//     }

//     return detectedBoxes;

// }




// std::string ObjectDetection::getClassLabel(int classId) {
//     switch (classId) {
//         case 0: return "MobileGoal";
//         case 1: return "RedRing";
//         case 2: return "BlueRing";
//         default: return "Unknown";
//     }
// }

// void ObjectDetection::printDecodedOutputs(const std::vector<BoundingBox>& detections) {
//     if (detections.empty()) {
//         std::cout << "No detections found!" << std::endl;
//         return;
//     }

//     std::cout << "==================== Decoded Outputs ====================" << std::endl;
//     std::cout << "Total Detections: " << detections.size() << std::endl;

//     for (size_t i = 0; i < detections.size(); i++) {
//         const BoundingBox& bbox = detections[i];

//         std::cout << "Detection " << i + 1 << ":\n";
//         std::cout << "  Class ID     : " << bbox.classId << " (" << getClassLabel(bbox.classId) << ")\n";
//         std::cout << "  Confidence   : " << bbox.confidence * 100 << "%\n";
//         std::cout << "  Bounding Box : ("
//                   << "X: " << bbox.x << ", Y: " << bbox.y 
//                   << ", Width: " << bbox.width << ", Height: " << bbox.height 
//                   << ")\n";
//         std::cout << "------------------------------------------------------\n";
//     }
// }





// ObjectDetection::ObjectDetection()
// {
// }





// std::vector<BoundingBox> ObjectDetection::applyNMS(std::vector<BoundingBox> boxes, float iouThreshold) {
//     std::vector<BoundingBox> finalBoxes;

//     // Sort boxes by confidence score
//     std::sort(boxes.begin(), boxes.end(), [](const BoundingBox& a, const BoundingBox& b) {
//         return a.confidence > b.confidence;
//     });

//     while (!boxes.empty()) {
//         BoundingBox best = boxes[0];
//         finalBoxes.push_back(best);
//         boxes.erase(boxes.begin());

//         auto iou = [](BoundingBox a, BoundingBox b) {
//             float x1 = std::max(a.x, b.x);
//             float y1 = std::max(a.y, b.y);
//             float x2 = std::min(a.x + a.width, b.x + b.width);
//             float y2 = std::min(a.y + a.height, b.y + b.height);
//             float interArea = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
//             float unionArea = a.width * a.height + b.width * b.height - interArea;
//             return interArea / unionArea;
//         };

//         boxes.erase(std::remove_if(boxes.begin(), boxes.end(), [&](BoundingBox b) {
//             return iou(best, b) > iouThreshold;
//         }), boxes.end());
//     }

//     return finalBoxes;
// }

// void ObjectDetection::mergeDetections(std::vector<BoundingBox> detections1, std::vector<BoundingBox> detections2)
// {
//     std::vector<BoundingBox> allDetections;
//     allDetections.insert(allDetections.end(), detections1.begin(), detections1.end());
//     allDetections.insert(allDetections.end(), detections2.begin(), detections2.end());

//     // Apply Non-Max Suppression (NMS) to remove overlapping detections
//     detections = applyNMS(allDetections, 0.5);
// }






// std::vector<BoundingBox> ObjectDetection::decodeOutputs(
//     const std::vector<std::vector<std::vector<float>>>& reshaped_output, int grid_w, int grid_h)
// {
//     std::vector<BoundingBox> detectedBoxes;
//     float original_width = 640;
//     float original_height = 480;
//     int mask_index = (grid_w == 10) ? 0 : 1;  // Select correct mask based on grid size

//     for (size_t i = 0; i < reshaped_output[0].size(); i++)  
//     {
//         float centerX_raw = reshaped_output[0][i][0];  
//         float centerY_raw = reshaped_output[0][i][1];

//         centerX_raw = std::max(std::min(centerX_raw, 10.0f), -10.0f);
//         centerY_raw = std::max(std::min(centerY_raw, 10.0f), -10.0f);

//         float centerX = 1 / (1 + exp(-centerX_raw));  
//         float centerY = 1 / (1 + exp(-centerY_raw));

//         int row = i / grid_w;
//         int col = i % grid_w;

//         if (centerX > 1.5 || centerY > 1.5) {  // 🚀 Only scale if necessary
//             centerX = (centerX + col) / grid_w;
//             centerY = (centerY + row) / grid_h;
//         }

//         int anchor_index = yolo_masks[mask_index][(i / (grid_w * grid_h)) % 3];
//         float anchor_w = anchors[anchor_index].first;
//         float anchor_h = anchors[anchor_index].second;

//         float bbox_w_raw = reshaped_output[0][i][2];
//         float bbox_h_raw = reshaped_output[0][i][3];

//         bbox_w_raw = std::max(std::min(bbox_w_raw, 10.0f), -10.0f);
//         bbox_h_raw = std::max(std::min(bbox_h_raw, 10.0f), -10.0f);

//         float bbox_width = std::max(std::min(expf(bbox_w_raw) * anchor_w, original_width), 1.0f);
//         float bbox_height = std::max(std::min(expf(bbox_h_raw) * anchor_h, original_height), 1.0f);

//         float confidence = 1 / (1 + exp(-reshaped_output[0][i][4]));
//         if (std::isnan(confidence) || std::isinf(confidence)) confidence = 0;
//         if (confidence < 0.3) continue;

//         float bestClassScore = 0;
//         int bestClassIndex = -1;

//         for (int c = 5; c < 8; c++) {  
//             float classProb = 1 / (1 + exp(-reshaped_output[0][i][c]));
//             if (classProb > bestClassScore) {
//                 bestClassScore = classProb;
//                 bestClassIndex = c - 5;
//             }
//         }

//         if (std::isnan(bestClassScore) || std::isinf(bestClassScore)) bestClassScore = 0;
//         float finalConfidence = confidence * bestClassScore;
//         if (finalConfidence < 0.3) continue;

//         centerX *= original_width;
//         centerY *= original_height;
//         bbox_width *= original_width;
//         bbox_height *= original_height;

//         BoundingBox bbox;
//         bbox.x = centerX - (bbox_width / 2);
//         bbox.y = centerY - (bbox_height / 2);
//         bbox.width = bbox_width;
//         bbox.height = bbox_height;
//         bbox.confidence = finalConfidence;
//         bbox.classId = bestClassIndex;

//         detectedBoxes.push_back(bbox);
//     }

//     return detectedBoxes;
// }




// std::vector<float> ObjectDetection::decodeOutputs(const std::vector<float>& output1, int grid_w1, int grid_h1,
//                                                   const std::vector<float>& output2, int grid_w2, int grid_h2) 
// {
    
//     std::vector<float> decodedOutputs;  // Flattened output vector

//     // Define tuple container for processing both outputs
//     std::vector<std::tuple<const std::vector<float>&, int, int, int>> outputs = {
//         std::make_tuple(output1, grid_w1, grid_h1, 0),  // First output (10x10)
//         std::make_tuple(output2, grid_w2, grid_h2, 1)   // Second output (20x20)
//     };

//     for (const std::tuple<const std::vector<float>&, int, int, int>& tuple : outputs) {
//         const std::vector<float>& output = std::get<0>(tuple);
//         int grid_w = std::get<1>(tuple);
//         int grid_h = std::get<2>(tuple);
//         int mask_index = std::get<3>(tuple);

//         int numCells = grid_w * grid_h;
//         int numAnchors = 3;
//         int numElements = 8;  // (x, y, w, h, conf, class1, class2, class3)

//         // Extract correct anchors
//         std::pair<float, float> selected_anchors[3];
//         for (int i = 0; i < 3; i++) {  
//             selected_anchors[i] = anchors[masks[mask_index][i]];
//         }

//         for (int y = 0; y < grid_h; y++) {
//             for (int x = 0; x < grid_w; x++) {
//                 for (int anchor = 0; anchor < 3; anchor++) {
                    
//                     int index = (anchor * grid_w * grid_h + y * grid_w + x) * 8;

//                     float confidence = 1 / (1 + exp(-output[index + 4]));
//                     if (confidence < 0.3) continue;

//                     float centerX = 1 / (1 + exp(-output[index + 0]));
//                     float centerY = 1 / (1 + exp(-output[index + 1]));
//                     float bbox_width = exp(output[index + 2]) * selected_anchors[anchor].first;
//                     float bbox_height = exp(output[index + 3]) * selected_anchors[anchor].second;
                    

                    

//                     float bestClassScore = 0;
//                     int bestClassIndex = -1;
//                     std::vector<float> classProbs;
//                     for (int c = 5; c < 8; c++) {  
//                         float classProb = 1 / (1 + exp(-output[index + c]));
//                         classProbs.push_back(classProb);

//                         float finalScore = confidence * classProb;
//                         if (finalScore > bestClassScore) {
//                             bestClassScore = finalScore;
//                             bestClassIndex = c - 5;
//                         }
//                     }

//                     if(bestClassIndex == -1 || bestClassScore < object_thresholds[bestClassIndex]) continue;

//                     decodedOutputs.push_back((centerX + x) / grid_w);
//                     decodedOutputs.push_back((centerY + y) / grid_h);
//                     decodedOutputs.push_back(bbox_width);
//                     decodedOutputs.push_back(bbox_height);
//                     decodedOutputs.push_back(bestClassScore);
//                     decodedOutputs.push_back(bestClassIndex);
//                 }
//             }
//         }
//     }
//     //printDecoded(decodedOutputs);
//     return decodedOutputs;

// }




// void Model::engineInfo()
// {
//     if(hasFP16)
//         cout << "The platform has FP16 native support" << endl;
//     else
//         cout << "The platform does not have FP16 native support" << endl;

//     cout << "The input buffer size is " << inputSize/sizeof(float) << endl;
//     // cout << "With a shape of ( ";
//     // for(int i = 0; i < inputDims.nbDims ;i++)
//     //     cout << inputDims.d[i] << " ";
//     // cout << ")" << endl;

//     cout << "The output 1 buffer size is " << outputSize1/sizeof(float) << endl;
//     // cout << "With a shape of ( ";
//     // for(int i = 0; i < outputDims1.nbDims ;i++)
//     //     cout << outputDims1.d[i] << " ";
//     // cout << ")" << endl;

//     cout << "The output 2 buffer size is " << outputSize2/sizeof(float) << endl;
//     // cout << "With a shape of ( ";
//     // for(int i = 0; i < outputDims2.nbDims ;i++)
//     //     cout << outputDims2.d[i] << " ";
//     // cout << ")" << endl;
// }




// void Model::saveRawOutput(const std::vector<float>& output, int grid_w, int grid_h, const std::string& filename)
// {
//     std::ofstream file(filename);
//     if(!file.is_open())
//     {
//         std::cerr << "Error opening file to save raw data" << std::endl;
//     }

//     int num_channels = output.size() / (grid_h * grid_w);

//     for(int c = 0; c < num_channels; c++)
//     {
//         for(int y = 0; y < grid_h; y++)
//         {
//             for(int x = 0; x < grid_w; x++)
//             {
//                 int index = c * grid_w * grid_h + y * grid_w + x;
//                 file << output[index] << ", ";
            
//             }
//             file << std::endl;
//         }   
//         file << std::endl;
//     }

//     file.close();
// }


// void Model::visualizeClassFeatureMaps(const std::vector<float>& inferOutput,int grid_size) 
// {
//     int num_anchors = 3;
//     int num_attrs = 8;  // x, y, w, h, confidence, class1, class2, class3

//     cv::Mat feature_map_class1(grid_size, grid_size, CV_32F, cv::Scalar(0));
//     cv::Mat feature_map_class2(grid_size, grid_size, CV_32F, cv::Scalar(0));
//     cv::Mat feature_map_class3(grid_size, grid_size, CV_32F, cv::Scalar(0));

//     // Extract confidence scores for each class from inferOutput2
//     for (int gy = 0; gy < grid_size; gy++) {
//         for (int gx = 0; gx < grid_size; gx++) {
//             float conf_class1 = 0, conf_class2 = 0, conf_class3 = 0;

//             for (int a = 0; a < num_anchors; a++) {
//                 int index = ((gy * grid_size + gx) * num_anchors + a) * num_attrs;
                
//                 // Multiply confidence by class probability to get final score
//                 float confidence = sigmoid(inferOutput1[index + 4]);
//                 float class1_prob = inferOutput[index + 5];  // MobileGoal
//                 float class2_prob = inferOutput[index + 6];  // RedRing
//                 float class3_prob = inferOutput[index + 7];  // BlueRing

//                 conf_class1 += confidence * class1_prob;
//                 conf_class2 += confidence * class2_prob;
//                 conf_class3 += confidence * class3_prob;

//             }

//             // Store confidence values for each class
//             feature_map_class1.at<float>(gy, gx) = conf_class1;
//             feature_map_class2.at<float>(gy, gx) = conf_class2;
//             feature_map_class3.at<float>(gy, gx) = conf_class3;

//         }
//     }

//     // Normalize and apply colormap
//     cv::Mat heatmap1, heatmap2, heatmap3;
//     cv::normalize(feature_map_class1, feature_map_class1, 0, 255, cv::NORM_MINMAX);
//     feature_map_class1.convertTo(feature_map_class1, CV_8U);
//     cv::applyColorMap(feature_map_class1, heatmap1, cv::COLORMAP_JET);

//     cv::normalize(feature_map_class2, feature_map_class2, 0, 255, cv::NORM_MINMAX);
//     feature_map_class2.convertTo(feature_map_class2, CV_8U);
//     cv::applyColorMap(feature_map_class2, heatmap2, cv::COLORMAP_JET);

//     cv::normalize(feature_map_class3, feature_map_class3, 0, 255, cv::NORM_MINMAX);
//     feature_map_class3.convertTo(feature_map_class3, CV_8U);
//     cv::applyColorMap(feature_map_class3, heatmap3, cv::COLORMAP_JET);

//     cv::resize(heatmap1,heatmap1,cv::Size(640,480),0,0,cv::INTER_LINEAR);
//     cv::resize(heatmap2,heatmap2,cv::Size(640,480),0,0,cv::INTER_LINEAR);
//     cv::resize(heatmap3,heatmap3,cv::Size(640,480),0,0,cv::INTER_LINEAR);
//     std::string window_suffix = " (" + std::to_string(grid_size) + "x" + std::to_string(grid_size) + ")";
//     // Display the heatmaps
//     cv::imshow("MobileGoal Confidence Map" + window_suffix, heatmap1);
//     cv::imshow("RedRing Confidence Map" + window_suffix, heatmap2);
//     cv::imshow("BlueRing Confidence Map" + window_suffix, heatmap3);
//     //cv::waitKey(0);
// }

// void Model::analyzeOutput(const std::vector<float>& output, int grid_h, int grid_w) 
// {
//     int num_anchors = 3;
//     int num_classes = 3;  // MobileGoal, RedRing, BlueRing
//     int num_attributes = 5 + num_classes;  // x,y,w,h,conf + classes
    
//     // Calculate overall statistics
//     float min_val = std::numeric_limits<float>::max();
//     float max_val = std::numeric_limits<float>::lowest();
//     float sum = 0.0f;
    
//     for(size_t i = 0; i < output.size(); i++) {
//         min_val = std::min(min_val, output[i]);
//         max_val = std::max(max_val, output[i]);
//         sum += output[i];
//     }
    
//     std::cout << "Raw output statistics:" << std::endl;
//     std::cout << "Min value: " << min_val << std::endl;
//     std::cout << "Max value: " << max_val << std::endl;
//     std::cout << "Mean value: " << sum/output.size() << std::endl;
//     std::cout << "Output size: " << output.size() << std::endl;

//     // Analyze confidence scores
//     std::vector<float> confidence_scores;
//     for(int y = 0; y < grid_h; y++) {
//         for(int x = 0; x < grid_w; x++) {
//             for(int a = 0; a < num_anchors; a++) {
//                 // Calculate base index for this anchor box
//                 int base_idx = (a * grid_h * grid_w + y * grid_w + x) * num_attributes;
                
//                 // Get confidence score (index 4 after x,y,w,h)
//                 float conf = sigmoid(output[base_idx + 4]);
//                 confidence_scores.push_back(conf);

//                 // If confidence is significant, print more details
//                 if(conf > 0.5) {
//                     std::cout << "\nHigh confidence detection at grid (" << x << "," << y 
//                               << "), anchor " << a << ":" << std::endl;
//                     std::cout << "Confidence: " << conf << std::endl;
                    
//                     // Print class probabilities
//                     for(int c = 0; c < num_classes; c++) {
//                         float class_prob = sigmoid(output[base_idx + 5 + c]);
//                         std::cout << "Class " << c << " prob: " << class_prob << std::endl;
//                     }
//                 }
//             }
//         }
//     }

//     // Analyze confidence score distribution
//     if(!confidence_scores.empty()) {
//         float conf_min = *std::min_element(confidence_scores.begin(), confidence_scores.end());
//         float conf_max = *std::max_element(confidence_scores.begin(), confidence_scores.end());
//         float conf_sum = accumulate(confidence_scores.begin(), confidence_scores.end(), 0.0f);
//         float conf_mean = conf_sum / confidence_scores.size();

//         std::cout << "\nConfidence score statistics:" << std::endl;
//         std::cout << "Min confidence: " << conf_min << std::endl;
//         std::cout << "Max confidence: " << conf_max << std::endl;
//         std::cout << "Mean confidence: " << conf_mean << std::endl;
//     }
// }


// float Model::sigmoid(float x) {
//     return 1.0f / (1.0f + exp(-x));
// }

// void Model::visualizeConfidenceDistribution(const std::vector<float>& output, int grid_size) {
//     std::vector<float> confidences;
//     int numElements = output.size() / 8;  // 8 values per grid cell
    
//     for (int i = 0; i < numElements; i++) {
//         float conf = sigmoid(output[i * 8 + 4]);
//         confidences.push_back(conf);
//     }
    
//     // Create histogram
//     const int NUM_BINS = 10;
//     std::vector<int> histogram(NUM_BINS, 0);
//     for (float conf : confidences) {
//         int bin = std::min(static_cast<int>(conf * NUM_BINS), NUM_BINS - 1);
//         histogram[bin]++;
//     }
    
//     // Print histogram
//     std::cout << "\nConfidence Distribution (" << grid_size << "x" << grid_size << "):" << std::endl;
//     for (int i = 0; i < NUM_BINS; i++) {
//         std::cout << "[" << i * 0.1 << "-" << (i + 1) * 0.1 << "]: ";
//         for (int j = 0; j < histogram[i] / 10; j++) std::cout << "*";
//         std::cout << " (" << histogram[i] << ")" << std::endl;
//     }
// }

// void Model::debugStrideMapping()
// {
//     // Create a test pattern
//     cv::Mat testImage = cv::Mat::zeros(320, 320, CV_8UC3);
    
//     // Draw markers at specific positions to track through network
//     cv::circle(testImage, cv::Point(80, 80), 5, cv::Scalar(255,0,0), -1);
//     cv::circle(testImage, cv::Point(160, 160), 5, cv::Scalar(0,255,0), -1);
//     cv::circle(testImage, cv::Point(240, 240), 5, cv::Scalar(0,0,255), -1);
    
//     // Process through your network
//     float* inputTensor = new float[3 * 320 * 320];
//     preprocessTestImage(testImage, inputTensor);
    
//     // Run inference and analyze feature map activations
//     context->enqueueV2(buffers.data(), stream, nullptr);
    
//     // Print highest activations in feature maps
//     analyzeFeatureMapActivations(inferOutput1, 10, "10x10 Grid");
//     analyzeFeatureMapActivations(inferOutput2, 20, "20x20 Grid");
    
//     delete[] inputTensor;
// }

// void Model::analyzeFeatureMapActivations(const std::vector<float>& output, int grid_size, const char* name)
// {
//     std::cout << "\nAnalyzing " << name << " activations:" << std::endl;
    
//     // For each grid cell
//     for(int y = 0; y < grid_size; y++) {
//         for(int x = 0; x < grid_size; x++) {
//             float maxConf = 0.0f;
//             // Check all anchors
//             for(int a = 0; a < 3; a++) {
//                 int index = (a * grid_size * grid_size + y * grid_size + x) * 8 + 4;
//                 float conf = sigmoid(output[index]);
//                 maxConf = std::max(maxConf, conf);
//             }
            
//             // Print if significant activation found
//             if(maxConf > 0.5) {
//                 std::cout << "High activation at grid (" << x << "," << y 
//                          << ") with confidence: " << maxConf << std::endl;
//             }
//         }
//     }
// }

// void Camera::preprocessFrames(float* InputData)
// {
//     const int target_height = 320;
//     const int target_width = 320;
//     const int channels = 3;

//     // 1. Print raw BGR values from original image
//     cv::Vec3b firstPixel = color_mat.at<cv::Vec3b>(0,0);
//     cv::Vec3b midPixel = color_mat.at<cv::Vec3b>(color_mat.rows/2, color_mat.cols/2);
//     std::cout << "\nOriginal BGR values:" << std::endl;
//     std::cout << "First pixel: [" << (int)firstPixel[0] << ", " << (int)firstPixel[1] << ", " << (int)firstPixel[2] << "]" << std::endl;
//     std::cout << "Mid pixel: [" << (int)midPixel[0] << ", " << (int)midPixel[1] << ", " << (int)midPixel[2] << "]" << std::endl;

//     // 2. Convert and resize
//     cv::Mat resized;
//     cv::cvtColor(color_mat, resized, cv::COLOR_BGR2RGB);
//     cv::resize(resized, resized, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);

//     // Print RGB values after conversion
//     cv::Vec3b firstPixelRGB = resized.at<cv::Vec3b>(0,0);
//     cv::Vec3b midPixelRGB = resized.at<cv::Vec3b>(target_height/2, target_width/2);
//     std::cout << "\nAfter BGR2RGB conversion:" << std::endl;
//     std::cout << "First pixel: [" << (int)firstPixelRGB[0] << ", " << (int)firstPixelRGB[1] << ", " << (int)firstPixelRGB[2] << "]" << std::endl;
//     std::cout << "Mid pixel: [" << (int)midPixelRGB[0] << ", " << (int)midPixelRGB[1] << ", " << (int)midPixelRGB[2] << "]" << std::endl;

//     // 3. Convert to float and normalize
//     resized.convertTo(resized, CV_32F, 1.0/255.0);

//     // Print normalized values
//     cv::Vec3f firstPixelNorm = resized.at<cv::Vec3f>(0,0);
//     cv::Vec3f midPixelNorm = resized.at<cv::Vec3f>(target_height/2, target_width/2);
//     std::cout << "\nAfter normalization [0,1]:" << std::endl;
//     std::cout << "First pixel: [" << firstPixelNorm[0] << ", " << firstPixelNorm[1] << ", " << firstPixelNorm[2] << "]" << std::endl;
//     std::cout << "Mid pixel: [" << midPixelNorm[0] << ", " << midPixelNorm[1] << ", " << midPixelNorm[2] << "]" << std::endl;

//     // 4. Reorder to CHW format and print final tensor values
//     int image_size = target_height * target_width;
//     for(int c = 0; c < channels; c++) {
//         for(int h = 0; h < target_height; h++) {
//             for(int w = 0; w < target_width; w++) {
//                 InputData[c * image_size + h * target_width + w] = resized.at<cv::Vec3f>(h, w)[c];
//             }
//         }
//     }

//     // Print final tensor values
//     std::cout << "\nFinal tensor values (first few):" << std::endl;
//     for(int c = 0; c < channels; c++) {
//         std::cout << "Channel " << c << " [0,0]: " << InputData[c * image_size] << std::endl;
//         std::cout << "Channel " << c << " [mid,mid]: " << 
//             InputData[c * image_size + (target_height/2) * target_width + target_width/2] << std::endl;
//     }

//     // Print value ranges
//     float min_val = std::numeric_limits<float>::max();
//     float max_val = std::numeric_limits<float>::lowest();
//     for(int i = 0; i < channels * image_size; i++) {
//         min_val = std::min(min_val, InputData[i]);
//         max_val = std::max(max_val, InputData[i]);
//     }
//     std::cout << "\nFinal tensor range: [" << min_val << ", " << max_val << "]" << std::endl;
// }




// cv::Mat Camera::preprocessFrames()
// {

//     // Resize to 320x320
//     cv::Mat resized;
//     cv::resize(color_mat, resized, cv::Size(320, 320));

//     // Convert BGR to HSV
//     cv::Mat hsv;
//     cv::cvtColor(resized, hsv, cv::COLOR_BGR2HSV);

//     // Apply Hue, Saturation, and Value adjustments (same as Python)
//     for (int y = 0; y < hsv.rows; y++) {
//         for (int x = 0; x < hsv.cols; x++) {
//             cv::Vec3b& pixel = hsv.at<cv::Vec3b>(y, x);
//             pixel[0] = pixel[0] + 0;  // Hue Shift
//             pixel[1] = cv::saturate_cast<uchar>(pixel[1] * 0);  // Saturation
//             pixel[2] = cv::saturate_cast<uchar>(pixel[2] * 0);  // Brightness
//         }
//     }

//     // Convert HSV back to RGB
//     cv::cvtColor(hsv, resized, cv::COLOR_HSV2RGB);

//     // Normalize to [0,1]
//     resized.convertTo(resized, CV_32F, 1.0 / 255.0);

//     // Convert from HWC to CHW
//     std::vector<cv::Mat> chw(3);
//     cv::split(resized, chw);

//     // Flatten into CHW vector
//     std::vector<float> chw_vector;
//     for (int i = 0; i < 3; i++) {
//         chw_vector.insert(chw_vector.end(), (float*)chw[i].datastart, (float*)chw[i].dataend);
//     }

//     cv::Mat output(1, 3 * 320 * 320, CV_32F, chw_vector.data());
//     return output;
// }


// void ObjectDetection::debugSpatialAlignment(const std::vector<float>& output, int grid_size)
// {
//     cv::Mat visualizer = cv::Mat::zeros(320, 320, CV_8UC3);
//     float cell_width = 320.0f / grid_size;
//     float cell_height = 320.0f / grid_size;
    
//     // Draw grid
//     for(int i = 0; i <= grid_size; i++) {
//         cv::line(visualizer, 
//                  cv::Point(i * cell_width, 0), 
//                  cv::Point(i * cell_width, 320), 
//                  cv::Scalar(50,50,50));
//         cv::line(visualizer, 
//                  cv::Point(0, i * cell_height), 
//                  cv::Point(320, i * cell_height), 
//                  cv::Scalar(50,50,50));
//     }
    
//     // Visualize detections and their grid alignment
//     for(int y = 0; y < grid_size; y++) {
//         for(int x = 0; x < grid_size; x++) {
//             for(int a = 0; a < 3; a++) {
//                 int index = (a * grid_size * grid_size + y * grid_size + x) * 8;
                
//                 float conf = sigmoid(output[index + 4]);
//                 if(conf > 0.3) {
//                     // Get box parameters
//                     float cx = (sigmoid(output[index + 0]) + x) * cell_width;
//                     float cy = (sigmoid(output[index + 1]) + y) * cell_height;
//                     float w = exp(output[index + 2]) * anchors[masks[grid_size == 10 ? 1 : 0][a]].first;
//                     float h = exp(output[index + 3]) * anchors[masks[grid_size == 10 ? 1 : 0][a]].second;
                    
//                     // Draw predicted box
//                     cv::rectangle(visualizer,
//                                 cv::Point(cx - w/2, cy - h/2),
//                                 cv::Point(cx + w/2, cy + h/2),
//                                 cv::Scalar(0,255*conf,0));
                    
//                     // Draw grid cell center
//                     cv::circle(visualizer,
//                              cv::Point((x + 0.5) * cell_width, (y + 0.5) * cell_height),
//                              2, cv::Scalar(0,0,255), -1);
//                 }
//             }
//         }
//     }
    
//     cv::imshow("Spatial Alignment Debug", visualizer);
//     cv::waitKey(1);
// }

// void ObjectDetection::debugAnchorBoxes(const std::vector<float>& output, int grid_size)
// {
//     std::cout << "\nAnchor box response analysis for " << grid_size << "x" << grid_size << " grid:" << std::endl;
    
//     // Track anchor usage
//     std::vector<int> anchor_usage(3, 0);
    
//     for(int y = 0; y < grid_size; y++) {
//         for(int x = 0; x < grid_size; x++) {
//             for(int a = 0; a < 3; a++) {
//                 int index = (a * grid_size * grid_size + y * grid_size + x) * 8;
//                 float conf = sigmoid(output[index + 4]);
                
//                 if(conf > 0.3) {
//                     anchor_usage[a]++;
                    
//                     // Print box dimensions for high confidence detections
//                     float w = exp(output[index + 2]) * anchors[masks[grid_size == 10 ? 1 : 0][a]].first;
//                     float h = exp(output[index + 3]) * anchors[masks[grid_size == 10 ? 1 : 0][a]].second;
                    
//                     std::cout << "Detection at (" << x << "," << y << ") using anchor " << a << ":\n"
//                               << "  Width: " << w << ", Height: " << h << "\n"
//                               << "  Confidence: " << conf << std::endl;
//                 }
//             }
//         }
//     }
    
//     // Print anchor usage statistics
//     std::cout << "\nAnchor usage statistics:" << std::endl;
//     for(int a = 0; a < 3; a++) {
//         std::cout << "Anchor " << a << ": " << anchor_usage[a] << " detections" << std::endl;
//     }
// }

// float ObjectDetection::sigmoid(float x) {
//     return 1.0f / (1.0f + exp(-x));
// }



// std::vector<float> ObjectDetection::decodeOutputs(const std::vector<float>& output, int grid_w, int grid_h, int mask_index) {
//     std::vector<float> decodedOutputs;

//     int numCells = grid_w * grid_h;
//     int numAnchors = 3;
//     int numElements = 8;  // (x, y, w, h, conf, class1, class2, class3)

//     // Extract correct anchors
//     std::pair<float, float> selected_anchors[3];
//     for (int i = 0; i < 3; i++) {  
//         selected_anchors[i] = anchors[masks[mask_index][i]];
//     }

//     for (int y = 0; y < grid_h; y++) {
//         for (int x = 0; x < grid_w; x++) {
//             std::cout << "Grid (" << x << ", " << y << "):";
//             for (int anchor = 0; anchor < 3; anchor++) {
                
//                 int index = (anchor * grid_w * grid_h + y * grid_w + x) * 8;

//                 float raw_confidence = output[index + 4];
//                 float confidence = 1 / (1 + exp(-raw_confidence));
//                 if (confidence < 0.3) continue;

//                 float centerX = 1 / (1 + exp(-output[index + 0]));
//                 float centerY = 1 / (1 + exp(-output[index + 1]));
//                 float bbox_width = exp(output[index + 2]) * selected_anchors[anchor].first;
//                 float bbox_height = exp(output[index + 3]) * selected_anchors[anchor].second;

//                 float bestClassScore = 0;
//                 int bestClassIndex = -1;
//                 for (int c = 5; c < 8; c++) {  
//                     float classProb = 1 / (1 + exp(-output[index + c]));
//                     float finalScore = confidence * classProb;

//                     if (finalScore > bestClassScore) {
//                         bestClassScore = finalScore;
//                         bestClassIndex = c - 5;
//                     }
//                 }

//                 if (bestClassIndex == -1 || bestClassScore < object_thresholds[bestClassIndex]) continue;

//                 decodedOutputs.push_back((centerX + x) / grid_w);
//                 decodedOutputs.push_back((centerY + y) / grid_h);
//                 decodedOutputs.push_back(bbox_width);
//                 decodedOutputs.push_back(bbox_height);
//                 decodedOutputs.push_back(bestClassScore);
//                 decodedOutputs.push_back(static_cast<float>(bestClassIndex));
//             }
//             std::cout << std::endl;
//         }
//     }
//     printDecoded(decodedOutputs);
//     return decodedOutputs;
// }



// void ObjectDetection::printDecoded(const std::vector<float>& decodedFlattened) {
//     std::cout << "==================== Decoded Flattened Output ====================" << std::endl;
//     std::cout << "Total Elements: " << decodedFlattened.size() << std::endl;

//     if (decodedFlattened.empty()) {
//         std::cout << "No detections found!" << std::endl;
//         return;
//     }

//     // Each detection has 6 values: [x, y, w, h, confidence, class]
//     for (size_t i = 0; i < decodedFlattened.size(); i += 6) {
//         std::cout << "Detection " << (i / 6) + 1 << " -> ";
//         std::cout << "X: " << decodedFlattened[i] << ", ";
//         std::cout << "Y: " << decodedFlattened[i + 1] << ", ";
//         std::cout << "W: " << decodedFlattened[i + 2] << ", ";
//         std::cout << "H: " << decodedFlattened[i + 3] << ", ";
//         std::cout << "Conf: " << decodedFlattened[i + 4] << ", ";
//         std::cout << "Class: " << decodedFlattened[i + 5] << std::endl;
//     }
                                                                                   
//     std::cout << "==================================================================" << std::endl;
// }






// std::vector<BoundingBox> ObjectDetection::applyNMS(std::vector<BoundingBox> boxes, float iouThreshold) {
//     std::vector<BoundingBox> finalBoxes;

//     std::sort(boxes.begin(), boxes.end(), [](const BoundingBox& a, const BoundingBox& b) 
//     {
//         return a.confidence > b.confidence;
//     });

//     while (!boxes.empty()) {
//         BoundingBox best = boxes[0];
//         finalBoxes.push_back(best);
//         boxes.erase(boxes.begin());

//         auto iou = [](BoundingBox a, BoundingBox b) {
//             float x1 = std::max(a.x, b.x);
//             float y1 = std::max(a.y, b.y);
//             float x2 = std::min(a.x + a.width, b.x + b.width);
//             float y2 = std::min(a.y + a.height, b.y + b.height);
//             float interArea = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
//             float unionArea = a.width * a.height + b.width * b.height - interArea;
//             return interArea / unionArea;
//         };

//         boxes.erase(std::remove_if(boxes.begin(), boxes.end(), [&](BoundingBox b) {
//             return iou(best, b) > iouThreshold;
//         }), boxes.end());
//     }

//     return finalBoxes;
// }









// // YOLO anchor boxes (predefined)
// std::vector<std::pair<float, float>> yolo_anchors = {
//     {10, 14}, {23, 27}, {37, 58}, {81, 82}, {135, 169}, {344, 319}
// };

// // YOLO masks for two output layers
// std::vector<std::vector<int>> yolo_masks = {
//     {3, 4, 5},  // First output layer (for large objects)
//     {0, 1, 2}   // Second output layer (for small objects)
// };

// // Function to process YOLO detections
// void processYoloDetections(const std::vector<float>& output, int gridH, int gridW, int img_width, int img_height, int maskIndex) {
//     int numAnchors = 3;
//     int numAttributes = 8; // X, Y, W, H, Confidence, Class1, Class2, Class3
//     std::vector<cv::Rect> boxes;
//     std::vector<float> confidences;
//     std::vector<int> class_ids;

//     // Select the correct anchor indices from the mask
//     std::vector<int> mask = yolo_masks[maskIndex];

//     for (int h = 0; h < gridH; h++) {
//         for (int w = 0; w < gridW; w++) {
//             for (int anchorIdx = 0; anchorIdx < numAnchors; anchorIdx++) {
//                 int anchor = mask[anchorIdx];  // Get the anchor index from the mask
//                 float anchor_w = yolo_anchors[anchor].first;
//                 float anchor_h = yolo_anchors[anchor].second;

//                 int baseChannel = anchorIdx * numAttributes;
//                 int index = ((baseChannel * gridH + h) * gridW) + w;

//                 // Extract raw model outputs
//                 float x_raw = output[index + 0];
//                 float y_raw = output[index + 1];
//                 float w_raw = output[index + 2];
//                 float h_raw = output[index + 3];
//                 float confidence = output[index + 4];

//                 // Apply sigmoid activation to x, y
//                 float x_center = (1 / (1 + exp(-x_raw)) + w) * (img_width / gridW);
//                 float y_center = (1 / (1 + exp(-y_raw)) + h) * (img_height / gridH);

//                 // Apply exponential scaling to width and height
//                 float width_abs = exp(w_raw) * anchor_w;
//                 float height_abs = exp(h_raw) * anchor_h;

//                 // Apply confidence threshold
//                 if (confidence < 0.5) continue;

//                 // Softmax for class scores
//                 float exp_sum = exp(output[index + 5]) + exp(output[index + 6]) + exp(output[index + 7]);
//                 float class1 = exp(output[index + 5]) / exp_sum;
//                 float class2 = exp(output[index + 6]) / exp_sum;
//                 float class3 = exp(output[index + 7]) / exp_sum;

//                 // Get best class
//                 std::vector<float> class_probs = {class1, class2, class3};
//                 int best_class = std::distance(class_probs.begin(), std::max_element(class_probs.begin(), class_probs.end()));

//                 // Convert to OpenCV rectangle format (x, y, w, h)
//                 cv::Rect bbox(x_center - width_abs / 2, y_center - height_abs / 2, width_abs, height_abs);
//                 boxes.push_back(bbox);
//                 confidences.push_back(confidence);
//                 class_ids.push_back(best_class);
//             }
//         }
//     }

//     // Apply Non-Maximum Suppression (NMS)
//     std::vector<int> indices;
//     cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.4, indices);

//     // Draw detections
//     cv::Mat image = cv::imread("image.jpg");
//     for (int i : indices) {
//         cv::rectangle(image, boxes[i], cv::Scalar(0, 255, 0), 2);
//         std::cout << "Detected Class: " << class_ids[i] << " at " << boxes[i] << std::endl;
//     }

//     cv::imshow("Detections", image);
//     cv::waitKey(0);
// }




// // Function to process YOLO detections
// void ObjectDetection::processDetections(const std::vector<float>& output, int gridH, int gridW, int maskIndex , std::vector<DetectedObject>& detections) 
// {
//     int numAnchors = 3;
//     int numAttributes = 8; // X, Y, W, H, Confidence, Class1, Class2, Class3

//     float scale_x = raw_width / img_width;
//     float scale_y = raw_height / img_height;

//     // Select the correct anchor indices from the mask
//     std::vector<int> mask = yolo_masks[maskIndex];

//     for (int h = 0; h < gridH; h++) 
//     {
//         for (int w = 0; w < gridW; w++) 
//         {
//             for (int anchorIdx = 0; anchorIdx < numAnchors; anchorIdx++) 
//             {
//                 int anchor = mask[anchorIdx];  // Get the anchor index from the mask
//                 float anchor_w = yolo_anchors[anchor].first;
//                 float anchor_h = yolo_anchors[anchor].second;

        
//                 int index = ((anchorIdx * numAttributes * gridH + h) * gridW + w);
//                 // Extract raw model outputs

//                 float confidence = sigmoid(output[index + 4]); 
//                 if (confidence < 0.3) continue;

//                 float class1 = sigmoid(output[index + 5]);
//                 float class2 = sigmoid(output[index + 6]);
//                 float class3 = sigmoid(output[index + 7]);

//                 // Get best class
//                 std::vector<float> class_probs = {class1, class2, class3};
//                 int best_class = std::distance(class_probs.begin(), std::max_element(class_probs.begin(), class_probs.end()));
                
//                 float best_confidence = class_probs[best_class] * confidence;
//                 if (best_confidence < object_thresholds[best_class]) continue;
                
//                 float x_raw = output[index + 0];
//                 float y_raw = output[index + 1];
//                 float w_raw = output[index + 2];
//                 float h_raw = output[index + 3];
                
//                 // Apply sigmoid activation to x, y (scaling inside the grid cell)
//                 float x_center = (sigmoid(x_raw) + w) * (raw_width / gridW);
//                 float y_center = (sigmoid(y_raw) + h) * (raw_height / gridH);
//                 // Apply exponential scaling to width and height using anchors
//                 float width = exp(w_raw) * anchor_w * scale_x;
//                 float height = exp(h_raw) * anchor_h * scale_y;
        

//                 float x = x_center - width / 2.0f;
//                 float y = y_center - height / 2.0f;


//                 // Convert to OpenCV rectangle format (x, y, w, h)
//                 cv::Rect box(x,y, width, height);

//                 DetectedObject Detected;
//                 Detected.bbox = box;
//                 Detected.confidence = confidence;
//                 Detected.classId = best_class;
//                 // Store the detection in the struct
//                 detections.push_back(Detected);
//             }
//         }
//     }
// }


// void ObjectDetection::processDetections(const std::vector<float>& output, int gridH, int gridW, int maskIndex, std::vector<DetectedObject>& detections) 
// {
//     int numAnchors = 3;
//     int numAttributes = 8; // X, Y, W, H, Confidence, Class1, Class2, Class3
//     float confidenceThreshold = 0.3; // Matches printAllGridCellValues() filtering

//     float scale_x = raw_width / img_width;
//     float scale_y = raw_height / img_height;

//     std::vector<int> mask = yolo_masks[maskIndex]; // Select correct anchor mask

//     for (int h = 0; h < gridH; h++) {
//         for (int w = 0; w < gridW; w++) {
//             for (int anchorIdx = 0; anchorIdx < numAnchors; anchorIdx++) {
//                 int anchor = mask[anchorIdx];  
//                 float anchor_w = yolo_anchors[anchor].first;
//                 float anchor_h = yolo_anchors[anchor].second;

//                 int index = ((anchorIdx * numAttributes + 0) * gridH + h) * gridW + w;


//                 float confidence = sigmoid(output[index + 4]); 
//                 if (confidence < confidenceThreshold) 
//                 {
//                     std::cout << "Skipped Confidecnce " << confidence << std::endl;
//                     continue; 
//                 }
//                 else
//                 {
//                     std::cout << "Passed Confidecnce " << confidence << std::endl;
//                 }

//                 float class1 = sigmoid(output[index + 5]);
//                 float class2 = sigmoid(output[index + 6]);
//                 float class3 = sigmoid(output[index + 7]);

//                 // Get best class
//                 std::vector<float> class_probs = {class1, class2, class3};
//                 int best_class = std::distance(class_probs.begin(), std::max_element(class_probs.begin(), class_probs.end()));
                
//                 float best_confidence = class_probs[best_class] * confidence;
//                 // if (best_confidence < object_thresholds[best_class]) continue;
                

//                 float x_raw = output[index + 0];
//                 float y_raw = output[index + 1];
//                 float w_raw = output[index + 2];
//                 float h_raw = output[index + 3];

        
//                 float x_center = (sigmoid(x_raw) + w) * (raw_width / gridW);
//                 float y_center = (sigmoid(y_raw) + h) * (raw_height / gridH);
//                 float width = exp(w_raw) * anchor_w * scale_x;
//                 float height = exp(h_raw) * anchor_h * scale_y;

//                 float x = x_center - width / 2.0f;
//                 float y = y_center - height / 2.0f;

                
//                 cv::Rect box(x, y, width, height);
//                 DetectedObject Detected;
//                 Detected.bbox = box;
//                 Detected.confidence = confidence;
//                 Detected.classId = best_class;
//                 detections.push_back(Detected);
//             }
//         }
//     }
// }




// bool Model::buildEngine() 
// {
//     cout << "Building engine this process may take a few minutes." << endl;
    
//     // Create builder and config first
//     unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(logger));
//     if (!builder) {
//         cerr << "Failed to create builder" << endl;
//         return false;
//     }

//     unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
//     if (!config) {
//         cerr << "Failed to create builder config" << endl;
//         return false;
//     }

//     // Set workspace size first - this affects what tactics are available
//     size_t maxWSsize = 1U << 29; // 512MB
//     config->setMaxWorkspaceSize(maxWSsize);

//     // Create network with explicit batch
//     const auto explicitBatch = 1U << static_cast<uint32_t>
//         (nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
//     unique_ptr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(explicitBatch));
//     if (!network) {
//         cerr << "Failed to create network" << endl;
//         return false;
//     }

//     // Parse ONNX early to allow modifications
//     cout << "Loading and parsing ONNX file: " << onnxPath << endl;
//     unique_ptr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*network, logger));
//     if (!parser) {
//         cerr << "Failed to create parser" << endl;
//         return false;
//     }

//     if (!parser->parseFromFile(onnxPath.c_str(), 
//         static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
//         cerr << "Failed to parse ONNX file: " << onnxPath << endl;
//         return false;
//     }

//     // Configure input dimensions explicitly
//     auto inputTensor = network->getInput(0);
//     if (!inputTensor) {
//         cerr << "Failed to get input tensor" << endl;
//         return false;
//     }
    
//     nvinfer1::Dims4 inputDims{1, 3, 320, 320};
//     inputTensor->setDimensions(inputDims);

//     // Optimize network configuration
//     config->setFlag(nvinfer1::BuilderFlag::kTF32);
    
//     // Check and enable FP16 if available
//     hasFP16 = builder->platformHasFastFp16();
//     if (hasFP16) {
//         cout << "Enabling FP16 precision" << endl;
//         config->setFlag(nvinfer1::BuilderFlag::kFP16);
//     }

//     // Set optimization profiles
//     IOptimizationProfile* profile = builder->createOptimizationProfile();
//     if (!profile) {
//         cerr << "Failed to create optimization profile" << endl;
//         return false;
//     }

//     // Set min, opt, max dimensions for input
//     profile->setDimensions(inputTensor->getName(), 
//         OptProfileSelector::kMIN, Dims4(1, 3, 320, 320));
//     profile->setDimensions(inputTensor->getName(), 
//         OptProfileSelector::kOPT, Dims4(1, 3, 320, 320));
//     profile->setDimensions(inputTensor->getName(), 
//         OptProfileSelector::kMAX, Dims4(1, 3, 320, 320));
    
//     config->addOptimizationProfile(profile);

//     // Build the engine
//     cout << "Building CUDA engine..." << endl;
//     unique_ptr<nvinfer1::ICudaEngine> localEngine(
//         builder->buildEngineWithConfig(*network, *config));
    
//     if (!localEngine) {
//         cerr << "Failed to build CUDA engine!" << endl;
//         return false;
//     }

//     // Serialize and save engine
//     cout << "Serializing engine..." << endl;
//     unique_ptr<nvinfer1::IHostMemory> serializedEngine(localEngine->serialize());
//     if (!serializedEngine) {
//         cerr << "Failed to serialize engine" << endl;
//         return false;
//     }

//     ofstream engineFile(enginePath, ios::binary);
//     if (!engineFile) {
//         cerr << "Failed to open engine file for writing" << endl;
//         return false;
//     }

//     engineFile.write(static_cast<const char*>(serializedEngine->data()), 
//         serializedEngine->size());
    
//     // Transfer ownership of the engine
//     engine = localEngine.release();
    
//     cout << "Engine built and saved successfully" << endl;
//     return true;
// }

// void Model::allocateBuffers() 
// {
//     if (!engine) {
//         throw runtime_error("Engine not initialized before buffer allocation");
//     }

//     cudaError_t err = cudaStreamCreate(&stream);
//     if (err != cudaSuccess) {
//         throw runtime_error("Failed to create CUDA stream: " + 
//             string(cudaGetErrorString(err)));
//     }

//     // Get binding indices
//     const vector<string> bindingNames = {
//         "000_net",
//         "016_convolutional",
//         "023_convolutional"
//     };

//     vector<int32_t> indices;
//     for (const auto& name : bindingNames) {
//         int idx = engine->getBindingIndex(name.c_str());
//         if (idx == -1) {
//             throw runtime_error("Failed to find binding: " + name);
//         }
//         indices.push_back(idx);
//     }

//     inputIndex = indices[0];
//     outputIndex1 = indices[1];
//     outputIndex2 = indices[2];

//     // Calculate sizes and allocate memory
//     auto calculateSize = [this](int32_t bindingIdx) -> size_t {
//         auto dims = engine->getBindingDimensions(bindingIdx);
//         size_t size = sizeof(float);
//         for (int i = 0; i < dims.nbDims; i++) {
//             size *= dims.d[i];
//         }
//         return size;
//     };

//     inputSize = calculateSize(inputIndex);
//     outputSize1 = calculateSize(outputIndex1);
//     outputSize2 = calculateSize(outputIndex2);

//     // Allocate host buffers
//     buffers.resize(engine->getNbBindings(), nullptr);
//     inferInput.resize(inputSize / sizeof(float));
//     inferOutput1.resize(outputSize1 / sizeof(float));
//     inferOutput2.resize(outputSize2 / sizeof(float));

//     // Allocate device memory
//     struct AllocationInfo {
//         void** ptr;
//         size_t size;
//     };
    
//     vector<AllocationInfo> allocations;
//     allocations.push_back({&buffers[inputIndex], inputSize});
//     allocations.push_back({&buffers[outputIndex1], outputSize1});
//     allocations.push_back({&buffers[outputIndex2], outputSize2});

//     for (const auto& alloc : allocations) {
//         err = cudaMalloc(alloc.ptr, alloc.size);
//         if (err != cudaSuccess) {
//             throw runtime_error("CUDA allocation failed: " + 
//                 string(cudaGetErrorString(err)));
//         }
//     }
// }




// void Model::printAllGridCellValues(const std::vector<float>& output, int gridH, int gridW) {
//     int C = 24; // Number of channels (3 anchors × 8 attributes)
//     int numAnchors = 3;
//     int numAttributes = 8; // 3 class probabilities + 1 confidence + 4 bbox

//     for (int h = 0; h < gridH; h++) {  // Iterate over grid rows (top to bottom)
//         for (int w = 0; w < gridW; w++) {  // Iterate over grid columns (left to right)
//             std::cout << "Grid cell (" << h << ", " << w << "):\n";

//             // Iterate over each anchor box in this grid cell
//             for (int anchor = 0; anchor < numAnchors; anchor++) {
//                 int baseChannel = anchor * numAttributes; // Each anchor starts every 8 channels

//                 std::cout << "  Anchor " << anchor + 1 << ":\n";
//                 for (int attr = 0; attr < numAttributes; attr++) {
//                     int channel = baseChannel + attr;

//                     // Compute index in NCHW format
//                     int index = ((channel * gridH + h) * gridW) + w;

//                     std::cout << "    Attribute " << attr << " (Channel " << channel << "): " 
//                               << sigmoid(output[index]) << "\n";
//                 }
//             }
//         }
//     }
// }





// float ObjectDetection::calculateIoU(const cv::Rect& box1, const cv::Rect& box2) {
//     cv::Rect intersection = box1 & box2;
//     float intersectionArea = intersection.area();
//     float unionArea = box1.area() + box2.area() - intersectionArea;
//     return unionArea > 0 ? intersectionArea / unionArea : 0;
// }



// void ObjectDetection::applyNMS(std::vector<DetectedObject>& detections) 
// {
//     std::cout << "\nPre-NMS detections: " << detections.size() << std::endl;
//     // Sort detections by confidence
//     std::sort(detections.begin(), detections.end(),
//               [](const DetectedObject& a, const DetectedObject& b) {
//                   return a.confidence > b.confidence;
//               });
    
//     std::vector<bool> keep(detections.size(), true);
//     int suppressed = 0;
    
//     for (size_t i = 0; i < detections.size(); ++i) {
//         if (!keep[i]) continue;
        
//         for (size_t j = i + 1; j < detections.size(); ++j) {
//             if (!keep[j]) continue;
            
//             if (detections[i].classId == detections[j].classId) {
//                 float iou = calculateIoU(detections[i].bbox, detections[j].bbox);
//                 if (iou > nms_threshold) {
//                     keep[j] = false;
//                     suppressed++;
//                 }
//             }
//         }
//     }
    
//     // Remove suppressed detections
//     std::vector<DetectedObject> filtered;
//     for (size_t i = 0; i < detections.size(); ++i) {
//         if (keep[i]) {
//             filtered.push_back(detections[i]);
//         }
//     }
//     std::cout << "Suppressed " << suppressed << " detections" << std::endl;
//     std::cout << "Post-NMS detections: " << filtered.size() << std::endl;
//     std::cout << "===========================================================\n\n" ;
//     detections = std::move(filtered);
// }


// void Model::printAllGridCellValues(const std::vector<float>& output, int gridH, int gridW) {
//     int C = 24; // Number of channels (3 anchors × 8 attributes)
//     int numAnchors = 3;
//     int numAttributes = 8; // 3 class probabilities + 1 confidence + 4 bbox

//     for (int h = 0; h < gridH; h++) {  // Iterate over grid rows (top to bottom)
//         for (int w = 0; w < gridW; w++) {  // Iterate over grid columns (left to right)
//             std::cout << "Grid cell (" << h << ", " << w << "):\n";

//             // Iterate over each anchor box in this grid cell
//             for (int anchor = 0; anchor < numAnchors; anchor++) {
//                 int baseChannel = anchor * numAttributes; // Each anchor starts every 8 channels

//                 std::cout << "  Anchor " << anchor + 1 << ":\n";
//                 for (int attr = 0; attr < numAttributes; attr++) {
//                     int channel = baseChannel + attr;

//                     // Compute index in NCHW format
//                     int index = ((channel * gridH + h) * gridW) + w;

//                     std::cout << "    Attribute " << attr << " (Channel " << channel << "): " 
//                               << sigmoid(output[index]) << "\n";
//                 }
//             }
//         }
//     }
// }

// void Model::printAllGridCellValues(const std::vector<float>& output, int gridH, int gridW) {
//     int C = 24; // Number of channels (3 anchors × 8 attributes)
//     int numAnchors = 3;
//     int numAttributes = 8; // 3 class probabilities + 1 confidence + 4 bbox
//     float threshold = 0.3;  // Confidence threshold

//     for (int h = 0; h < gridH; h++)
//     { 
//         for (int w = 0; w < gridW; w++) 
//         { 
//             bool hasPrintedCell = false;
//             for (int anchor = 0; anchor < numAnchors; anchor++) 
//             {
//                 int baseChannel = anchor * numAttributes;
//                 int confidenceIndex = ((baseChannel + 4) * gridH + h) * gridW + w;
//                 float confidence = sigmoid(output[confidenceIndex]);
//                 if (confidence < threshold)
//                 {
//                     continue;
//                 }
//                 if (!hasPrintedCell)
//                 {
//                     std::cout << "Grid cell (" << h << ", " << w << "):\n";
//                     hasPrintedCell = true;
//                 }
//                 std::cout << "  Anchor " << anchor + 1 << "(Confidence: " << confidence << "):\n";
    
//                 for (int attr = 0; attr < numAttributes; attr++)
//                 {
//                     int channel = baseChannel + attr;
//                     int index = ((channel * gridH + h) * gridW) + w;
//                     std::cout << "    Attribute " << attr << " (Channel " << channel << "): " << (sigmoid(output[index]) )<< "\n";
                   
    
//                 }
//             }
//         }
//     }
// }




// void Model::printConfidenceValues(const std::vector<float>& output, int gridH, int gridW) {
//     int C = 24; // Number of channels (3 anchors × 8 attributes)
//     int numAnchors = 3;
//     int numAttributes = 8; // 3 class probabilities + 1 confidence + 4 bbox
//     float threshold = 0.3 ;
//     int highConf = 0 ;

//     for (int h = 0; h < gridH; h++) {  // Iterate over grid rows (top to bottom)
//         for (int w = 0; w < gridW; w++) {  // Iterate over grid columns (left to right)
//             // std::cout << "Grid cell (" << h << ", " << w << "):\n";

//             // Iterate over each anchor box in this grid cell
//             for (int anchor = 0; anchor < numAnchors; anchor++) {
//                 int confidenceChannel = anchor * numAttributes + 4; // Confidence is attribute 4

//                 // Compute index in NCHW format
//                 int index = ((confidenceChannel * gridH + h) * gridW) + w;
//                 float confidence = sigmoid(output[index]);
//                 // std::cout << "  Anchor " << anchor + 1 << " - Confidence: " << confidence << "\n";
                
//                 if(confidence > threshold)
//                 {
//                     highConf++;
//                 }
//             }
//         }
//     }

//     std::cout << "\nTotal Confidence values above " << threshold << ": " << highConf << endl;
// }









// bool Model::buildEngine() 
// {
//     cout << "Building engine this process may take a few minutes." << endl;
    
//     // Create builder and config first
//     unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(logger));
//     if (!builder) {
//         cerr << "Failed to create builder" << endl;
//         return false;
//     }

//     unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
//     if (!config) {
//         cerr << "Failed to create builder config" << endl;
//         return false;
//     }

//     // Set workspace size first - this affects what tactics are available
//     size_t maxWSsize = 1U << 29; // 512MB
//     config->setMaxWorkspaceSize(maxWSsize);

//     // Create network with explicit batch
//     const auto explicitBatch = 1U << static_cast<uint32_t>
//         (nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
//     unique_ptr<nvinfer1::INetworkDefinition> network(builder->createNetworkV2(explicitBatch));
//     if (!network) {
//         cerr << "Failed to create network" << endl;
//         return false;
//     }

//     // Parse ONNX early to allow modifications
//     cout << "Loading and parsing ONNX file: " << onnxPath << endl;
//     unique_ptr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*network, logger));
//     if (!parser) {
//         cerr << "Failed to create parser" << endl;
//         return false;
//     }

//     if (!parser->parseFromFile(onnxPath.c_str(), 
//         static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
//         cerr << "Failed to parse ONNX file: " << onnxPath << endl;
//         return false;
//     }

//     // Configure input dimensions explicitly
//     auto inputTensor = network->getInput(0);
//     if (!inputTensor) {
//         cerr << "Failed to get input tensor" << endl;
//         return false;
//     }
    
//     nvinfer1::Dims4 inputDims{1, 3, 320, 320};
//     inputTensor->setDimensions(inputDims);

//     // Optimize network configuration
//     config->setFlag(nvinfer1::BuilderFlag::kTF32);
    

//     // Set optimization profiles
//     IOptimizationProfile* profile = builder->createOptimizationProfile();
//     if (!profile) {
//         cerr << "Failed to create optimization profile" << endl;
//         return false;
//     }

//     // Set min, opt, max dimensions for input
//     profile->setDimensions(inputTensor->getName(), 
//         OptProfileSelector::kMIN, Dims4(1, 3, 320, 320));
//     profile->setDimensions(inputTensor->getName(), 
//         OptProfileSelector::kOPT, Dims4(1, 3, 320, 320));
//     profile->setDimensions(inputTensor->getName(), 
//         OptProfileSelector::kMAX, Dims4(1, 3, 320, 320));
    
//     config->addOptimizationProfile(profile);

//     // Build the engine
//     cout << "Building CUDA engine..." << endl;
//     unique_ptr<nvinfer1::ICudaEngine> localEngine(
//         builder->buildEngineWithConfig(*network, *config));
    
//     if (!localEngine) {
//         cerr << "Failed to build CUDA engine!" << endl;
//         return false;
//     }

//     // Serialize and save engine
//     cout << "Serializing engine..." << endl;
//     unique_ptr<nvinfer1::IHostMemory> serializedEngine(localEngine->serialize());
//     if (!serializedEngine) {
//         cerr << "Failed to serialize engine" << endl;
//         return false;
//     }

//     ofstream engineFile(enginePath, ios::binary);
//     if (!engineFile) {
//         cerr << "Failed to open engine file for writing" << endl;
//         return false;
//     }

//     engineFile.write(static_cast<const char*>(serializedEngine->data()), 
//         serializedEngine->size());
    
//     // Transfer ownership of the engine
//     engine = localEngine.release();
    
//     cout << "Engine built and saved successfully" << endl;
//     return true;
// }

// void Model::allocateBuffers() 
// {
//     if (!engine) {
//         throw runtime_error("Engine not initialized before buffer allocation");
//     }

//     cudaError_t err = cudaStreamCreate(&stream);
//     if (err != cudaSuccess) {
//         throw runtime_error("Failed to create CUDA stream: " + 
//             string(cudaGetErrorString(err)));
//     }

//     // Get binding indices
//     const vector<string> bindingNames = {
//         "000_net",
//         "016_convolutional",
//         "023_convolutional"
//     };

//     vector<int32_t> indices;
//     for (const auto& name : bindingNames) {
//         int idx = engine->getBindingIndex(name.c_str());
//         if (idx == -1) {
//             throw runtime_error("Failed to find binding: " + name);
//         }
//         indices.push_back(idx);
//     }

//     inputIndex = indices[0];
//     outputIndex1 = indices[1];
//     outputIndex2 = indices[2];

//     // Calculate sizes and allocate memory
//     auto calculateSize = [this](int32_t bindingIdx) -> size_t {
//         auto dims = engine->getBindingDimensions(bindingIdx);
//         size_t size = sizeof(float);
//         for (int i = 0; i < dims.nbDims; i++) {
//             size *= dims.d[i];
//         }
//         return size;
//     };

//     inputSize = calculateSize(inputIndex);
//     outputSize1 = calculateSize(outputIndex1);
//     outputSize2 = calculateSize(outputIndex2);

//     // Allocate host buffers
//     buffers.resize(engine->getNbBindings(), nullptr);
//     inferInput.resize(inputSize / sizeof(float));
//     inferOutput1.resize(outputSize1 / sizeof(float));
//     inferOutput2.resize(outputSize2 / sizeof(float));

//     // Allocate device memory
//     struct AllocationInfo {
//         void** ptr;
//         size_t size;
//     };
    
//     vector<AllocationInfo> allocations;
//     allocations.push_back({&buffers[inputIndex], inputSize});
//     allocations.push_back({&buffers[outputIndex1], outputSize1});
//     allocations.push_back({&buffers[outputIndex2], outputSize2});

//     for (const auto& alloc : allocations) {
//         err = cudaMalloc(alloc.ptr, alloc.size);
//         if (err != cudaSuccess) {
//             throw runtime_error("CUDA allocation failed: " + 
//                 string(cudaGetErrorString(err)));
//         }
//     }
// }




// class Position {
// public:
//     // Status flags
//     static constexpr uint32_t STATUS_CONNECTED    = 0x00000001;
//     static constexpr uint32_t STATUS_NODOTS       = 0x00000002;
//     static constexpr uint32_t STATUS_NORAWBITS    = 0x00000004;
//     static constexpr uint32_t STATUS_NOGROUPS     = 0x00000008;
//     static constexpr uint32_t STATUS_NOBITS       = 0x00000010;
//     static constexpr uint32_t STATUS_PIXELERROR   = 0x00000020;
//     static constexpr uint32_t STATUS_SOLVER       = 0x00000040;
//     static constexpr uint32_t STATUS_ANGLEJUMP    = 0x00000080;
//     static constexpr uint32_t STATUS_POSJUMP      = 0x00000100;
//     static constexpr uint32_t STATUS_NOSOLUTION   = 0x00000200;
//     static constexpr uint32_t STATUS_KALMAN_EST   = 0x00100000;

//     uint32_t status;
//     float x, y, z, azimuth, elevation, rotation;
//     std::chrono::system_clock::time_point timestamp;

//     Position();
//     Position(uint32_t status, float x, float y, float z, float azimuth, float elevation, float rotation);

//     void printPosition() const;  // Add this to the public section of Position class
//     std::string getStatusString() const;  // Get status as readable string
//     std::string getStatusHex() const;     // Get status as hex value
    
//     // Added utility methods
//     bool isValid() const;
//     float distanceTo(const Position& other) const;
// };


// #ifndef BRAIN_H
// #define BRAIN_H

// #include <boost/asio.hpp>
// #include <boost/asio/read.hpp>
// #include <boost/asio/write.hpp>
// #include <thread>
// #include <mutex>
// #include <vector>
// #include <iostream>
// #include <atomic>
// #include <cstring>
// #include <memory>
// #include <chrono>
// #include <BrainPacket.h>


// class Brain {
// public:
//     explicit Brain(boost::asio::io_service& io_service, const std::string& port);
//     ~Brain();

//     bool start();
//     void stop();
//     bool restart();

//     bool isConnected() const { return connected_; }
//     bool isRunning() const { return running_; }
//     void sendRequest(uint16_t requestFlags);
    

// private:
//     void readLoop();
//     bool initializePort();
//     bool reconnect();
//     void processBrainRequest(const std::vector<uint8_t>& buffer);
//     void parseBrainResponse(const std::vector<uint8_t>& buffer);

//     std::string port_;
//     boost::asio::io_service& ioService;
//     std::unique_ptr<boost::asio::serial_port> serialPort;
//     std::mutex comms_mutex_;
//     std::atomic<bool> running_{false};
//     std::atomic<bool> connected_{false};
//     std::unique_ptr<std::thread> read_thread_;

//     const std::chrono::milliseconds READ_TIMEOUT{500};
//     const std::chrono::milliseconds RECONNECT_DELAY{2500};
// };

// #endif