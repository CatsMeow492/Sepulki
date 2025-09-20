import type { Metadata } from "next";
import { Inter } from "next/font/google";
import "./globals.css";
import Link from "next/link";
import { AuthProvider } from "@/components/AuthProvider";
import { SmithProfile } from "@/components/SmithProfile";

const inter = Inter({ subsets: ["latin"] });

export const metadata: Metadata = {
  title: "Sepulki Forge - Robotics as a Service",
  description: "Design, forge, and deploy robots with Sepulki's comprehensive robotics platform",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <body className={inter.className}>
        <AuthProvider>
          <div className="min-h-screen bg-gray-50">
            <nav className="bg-white shadow-sm">
              <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
                <div className="flex justify-between h-16">
                  <div className="flex">
                    <div className="flex-shrink-0 flex items-center">
                      <Link href="/" className="text-2xl font-bold text-orange-600">
                        🔥 Sepulki
                      </Link>
                    </div>
                    <div className="hidden sm:ml-6 sm:flex sm:space-x-8">
                      <Link
                        href="/configure"
                        className="text-gray-900 inline-flex items-center px-1 pt-1 border-b-2 border-transparent hover:border-orange-500"
                      >
                        Forge Robot
                      </Link>
                      <Link
                        href="/designs"
                        className="text-gray-900 inline-flex items-center px-1 pt-1 border-b-2 border-transparent hover:border-orange-500"
                      >
                        My Designs
                      </Link>
                      <Link
                        href="/dashboard"
                        className="text-gray-900 inline-flex items-center px-1 pt-1 border-b-2 border-transparent hover:border-orange-500"
                      >
                        Fleet Dashboard
                      </Link>
                      <Link
                        href="/pricing"
                        className="text-gray-900 inline-flex items-center px-1 pt-1 border-b-2 border-transparent hover:border-orange-500"
                      >
                        Pricing
                      </Link>
                    </div>
                  </div>
                  <div className="flex items-center space-x-4">
                    <SmithProfile />
                    <Link
                      href="/auth/signin"
                      className="bg-orange-600 text-white px-4 py-2 rounded-md text-sm font-medium hover:bg-orange-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-orange-500"
                    >
                      Get Started
                    </Link>
                  </div>
                </div>
              </div>
            </nav>
            <main>{children}</main>
            <footer className="bg-white border-t">
              <div className="max-w-7xl mx-auto py-6 px-4 sm:px-6 lg:px-8">
                <p className="text-center text-sm text-gray-500">
                  © 2024 Sepulki. All rights reserved.
                </p>
              </div>
            </footer>
          </div>
        </AuthProvider>
      </body>
    </html>
  );
}
