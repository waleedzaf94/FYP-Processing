using Microsoft.WindowsAzure.Storage;
using Microsoft.WindowsAzure.Storage.Blob;
using System;
using System.Collections.Generic;
using System.Configuration;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ManagedProcessing
{
    public static class StorageWrappers
    {
        public static CloudBlobClient CreateCloudBlobClient(CloudStorageAccount storageAccount)
        {
            return storageAccount.CreateCloudBlobClient();
        }


        public static CloudBlobContainer CreateContainerReference(CloudBlobClient cloudBlobClient, string container)
        {
            CloudBlobContainer cloudBlobContainer = cloudBlobClient.GetContainerReference(container);
            if (!cloudBlobContainer.Exists())
            {
                cloudBlobContainer.Create();
                Console.WriteLine("Created container '{0}'", cloudBlobContainer.Name);
                Console.WriteLine();
                // Set the permissions so the blobs are public. 
                BlobContainerPermissions permissions = new BlobContainerPermissions
                {
                    PublicAccess = BlobContainerPublicAccessType.Blob
                };
                cloudBlobContainer.SetPermissions(permissions);
            }
            return cloudBlobContainer;
        }

        internal static CloudBlobClient CreateCloudBlobClient()
        {
            throw new NotImplementedException();
        }
    }
}
