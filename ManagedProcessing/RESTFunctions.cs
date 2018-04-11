using System.Configuration;
using System.Linq;
using System.Net;
using System.Net.Http;
using System.Threading.Tasks;
using ManagedProcessing;
using Microsoft.Azure.WebJobs;
using Microsoft.Azure.WebJobs.Extensions.Http;
using Microsoft.Azure.WebJobs.Host;
using Microsoft.WindowsAzure.Storage;
using Microsoft.WindowsAzure.Storage.Blob;

namespace ProcessingWrapper
{
    public static class RESTFunctions
    {
        [FunctionName("RESTFunctions")]
        public static async Task<HttpResponseMessage> Run([HttpTrigger(AuthorizationLevel.Anonymous, "get", Route = null)]HttpRequestMessage req, TraceWriter log)
        {
            log.Info("C# HTTP trigger function processed a request.");

            // parse query parameter
            string containername = req.GetQueryNameValuePairs()
                .FirstOrDefault(q => string.Compare(q.Key, "container-name", true) == 0)
                .Value;

            // Set name to query string or body data

            string connectionString = ConfigurationManager.AppSettings["AzureWebJobsDashboard"];
            CloudStorageAccount storageAccount;
            CloudStorageAccount.TryParse(connectionString, out storageAccount);
            CloudBlobClient cloudBlobClient = StorageWrappers.CreateCloudBlobClient(storageAccount);
            CloudBlobContainer cloudBlobContainer = StorageWrappers.CreateContainerReference(cloudBlobClient, containername);



            return containername == null
                ? req.CreateResponse(HttpStatusCode.BadRequest, "Please pass a name on the query string or in the request body")
                : req.CreateResponse(HttpStatusCode.OK, "Hello " + containername);
        }
    }
}
