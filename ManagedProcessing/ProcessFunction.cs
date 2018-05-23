using System;
using System.Configuration;
using System.IO;
using System.Threading.Tasks;
using Microsoft.Azure.WebJobs;
using Microsoft.Azure.WebJobs.Host;
using Microsoft.WindowsAzure.Storage;
using Microsoft.WindowsAzure.Storage.Blob;

namespace ManagedProcessing
{
    public static class ProcessFunction
    {
        [FunctionName("ProcessFunction")]
        public async static void Run([BlobTrigger("recorded-objects/{name}", Connection = "AzureWebJobsStorage")]CloudBlockBlob myBlob, string name, TraceWriter log)
        {
            string container = "recorded-objects";
            log.Info($"C# Blob trigger function Processed blob\n Name:{name} \n Type: {myBlob.BlobType}");
            MeshProcessor processor = new MeshProcessor
            {
                InputBlob = name,
                InputContainer = container,
                OutputContainer = "output-objects",
                OutputName = name,
                OutputFilePath = Path.GetTempPath(),
                Log = log
            };
            //await  processor.RunAllTasksAsync();
            await CopyBlob(myBlob, log);
        }

        //private static string SaveTempFile(CloudBlob myBlob, string name, Processing.CommandWrap processor, TraceWriter log)
        //{
        //    string fileName = name;
        //    string tempPath = Path.GetTempPath();
        //    string filePath = tempPath + fileName;
        //    log.Info("Saving temporary file...", filePath);
        //    //await myBlob.BeginDownloadToFile(filePath, FileMode.CreateNew);
        //    processor.SetInput(filePath);
        //    log.Info("Input Data Set");
        //    return filePath;
        //}

        private async static Task CopyBlob(CloudBlockBlob myBlob, TraceWriter log)
        {
            var outputConatiner = "output-objects";
            var connectionString = ConfigurationManager.AppSettings["AzureWebJobsDashboard"];
            CloudStorageAccount.TryParse(connectionString, out CloudStorageAccount storageAccount);

            // Create the destination blob client
            CloudBlobClient blobClient = storageAccount.CreateCloudBlobClient();
            CloudBlobContainer container = blobClient.GetContainerReference(outputConatiner);
            // Create the container if it doesn't already exist.
            try
            {
                await container.CreateIfNotExistsAsync();
            }
            catch (Exception e)
            {
                log.Error(e.Message);
            }

            // Get hold of the destination blob
            CloudBlockBlob blockBlob = container.GetBlockBlobReference(myBlob.Name);
            log.Info($"BlockBlob destination name: {blockBlob.Name}");

            log.Info("Starting Copy");
            try
            {
                // Use this method to copy to blob 
                blockBlob.BeginStartCopy(myBlob, null, null);

                // alternatively, uncomment the code below and use this method to copy the blob data
                /*using (var stream = await myBlob.OpenReadAsync())
                {
                  await blockBlob.UploadFromStreamAsync(stream);
                }*/
                log.Info("Copy completed");

            }
            catch (Exception ex)
            {
                log.Error(ex.Message);
                log.Info("Copy failed");
            }
            finally
            {
                log.Info("Operation completed");
            }
        }
    }
}
