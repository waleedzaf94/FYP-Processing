using Microsoft.Azure.WebJobs.Host;
using Microsoft.WindowsAzure.Storage;
using Microsoft.WindowsAzure.Storage.Blob;
using System;
using System.Configuration;
using System.IO;
using System.Threading.Tasks;

namespace ManagedProcessing
{

    class MeshProcessor
    {
        Processing.CommandWrap processor;
        string inputContainer;
        string inputBlob;
        string connectionString;
        CloudBlobClient cloudBlobClient;
        CloudStorageAccount storageAccount;
        string outputName;
        private string LocalPath;

        public MeshProcessor()
        {
            connectionString = ConfigurationManager.AppSettings["AzureWebJobsDashboard"];
            CloudStorageAccount.TryParse(connectionString, out storageAccount);
            cloudBlobClient = StorageWrappers.CreateCloudBlobClient(storageAccount);
        }

        public string InputContainer { get => inputContainer; set => inputContainer = value; }
        public string InputBlob { get => inputBlob; set => inputBlob = value; }
        public TraceWriter Log { get; internal set; }
        public string OutputContainer { get; set; } = "output-objects";
        public string OutputName { get => outputName; set => outputName = value; }
        public string OutputFilePath { get; set; } = Path.GetTempPath();

        internal async Task RunAllTasksAsync()
        {
            Log.Info("Running Tasks...");
            await SaveMeshToTempAsync();
            Log.Info(LocalPath);
            CallProcessor();
            await UploadMeshToBlobAsync();
        }

        private void CallProcessor()
        {
            processor = new Processing.CommandWrap();
            //processor.Hello();
            processor.SetInput(LocalPath);
            processor.SetOutput(OutputName, OutputFilePath);
            processor.ProcessModel(true, "");
            processor.SaveFinalModel();
        }


        // Working
        private async Task SaveMeshToTempAsync()
        {
            CloudBlobContainer cloudBlobContainer = StorageWrappers.CreateContainerReference(cloudBlobClient, inputContainer);
            try
            {
                CloudBlockBlob cloudBlockBlob = cloudBlobContainer.GetBlockBlobReference(inputBlob);
                string tempPath = Path.GetTempPath();
                LocalPath = Path.Combine(tempPath, inputBlob);
                Log.Info($"Downloading blob to {LocalPath}");
                await cloudBlockBlob.DownloadToFileAsync(LocalPath, FileMode.Create);
            }
            catch (StorageException ex) {
                Log.Info("Error returned from the service: {0}", ex.Message);
            }
            //Log.Info(
            //    "A connection string has not been defined in the system environment variables. " +
            //    "Add a environment variable named 'storageconnectionstring' with your storage " +
            //    "connection string as a value.");
            
        }

        // Working!!
        private async Task UploadMeshToBlobAsync()
        {
            CloudBlobContainer cloudBlobContainer = StorageWrappers.CreateContainerReference(cloudBlobClient, OutputContainer);
            try
            {
                CloudBlockBlob cloudBlockBlob = cloudBlobContainer.GetBlockBlobReference(outputName);
                string sourceFile = Path.Combine(OutputFilePath, OutputName);
                Log.Info($"Uploading blob from {sourceFile}");
                await cloudBlockBlob.UploadFromFileAsync(sourceFile);
            }
            catch (StorageException ex)
            {
                Log.Info("Error returned from the service: {0}", ex.Message);
            }
        }

    }
}
