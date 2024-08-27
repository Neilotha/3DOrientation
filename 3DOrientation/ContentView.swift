//
//  ContentView.swift
//  3DOrientation
//
//  Created by Joshua Yang on 2023/3/20.
//

import SwiftUI

struct ContentView: View {
    @StateObject var model = OrientationModel()
    
    var body: some View {
        VStack {
            Image(systemName: "globe")
                .imageScale(.large)
                .foregroundColor(.accentColor)
            Text("Hello, world!")
            Button("Test") {
                print("test")
            }
        }
        .padding()
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
